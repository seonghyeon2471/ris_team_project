import cv2
import serial
import numpy as np
import time
import math
import threading

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# CAMERA & HARDWARE FIX
# =========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

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
EMA_ALPHA = 0.35
MEDIAN_K  = 2

_scan_buf    = np.full(360, 150.0, dtype=np.float32)
_scan_shared = np.full(360, 150.0, dtype=np.float32)
scan_lock    = threading.Lock()

# =========================================
# 카메라 기하학 파라미터
# =========================================
CAM_H        = 73.0
CAM_PITCH    = 30.0
CAM_FOV_V    = 50.0
RES_W, RES_H = 320, 240
CY           = RES_H / 2
FY           = (RES_H / 2) / math.tan(math.radians(CAM_FOV_V / 2))

def pixel_to_ground_dist(y_px):
    delta_deg  = math.degrees(math.atan2(y_px - CY, FY))
    total_deg  = CAM_PITCH + delta_deg
    if total_deg <= 0:
        return float('inf')
    return CAM_H / math.tan(math.radians(total_deg))

# =========================================
# 제어 및 주행 파라미터
# =========================================
MAX_V       = 0.24
MIN_V       = 0.10
KP_ROT      = 0.003
MIN_AREA    = 900
TARGET_AREA = 6000

APPROACH_V           = 0.24
BLIND_V              = 0.29

APPROACH_DRIVE_SEC   = 1.2
APPROACH_MAX_TIMEOUT = 4.5
SEARCH_TIMEOUT       = 1.6

PARK_SEC        = 3.0

# 바운더리 파라미터
BOUNDARY_PHASE1_SEC = 1.0
BOUNDARY_PHASE2_SEC = 1.8
BOUNDARY_PHASE3_SEC = 0.8
BOUNDARY_W          = 0.55
BOUNDARY_BACK_V     = -0.10

# =========================================
# 정렬(Align) 파라미터  ← 신규 추가
# =========================================
ALIGN_THRESHOLD = 30    # 이 픽셀 이내면 "정렬됨"으로 판단, 전진 허용
ALIGN_SCALE     = 1.5   # 정렬 안 됐을 때 회전력 배율

# =========================================
# HSV & BGR 정밀 컬러 지정
# =========================================
COLOR_CFG = {
    "red": {
        "hsv1": ([0, 60, 120], [12, 255, 255]),
        "hsv2": ([160, 60, 120], [179, 255, 255]),
        "bgr": ([20, 20, 80], [255, 255, 255]),
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv1": ([18, 15, 180], [45, 255, 255]),
        "hsv2": None,
        "bgr": ([0, 80, 80], [255, 255, 255]),
        "draw": (0, 200, 255),
    },
    "blue": {
        "hsv1": ([95, 50, 50], [130, 255, 255]),
        "hsv2": None,
        "bgr": ([40, 0, 0], [255, 220, 220]),
        "draw": (255, 80, 0),
    },
}

MISSION = ["red", "yellow", "blue"]

# =========================================
# 상태 변수
# =========================================
mission_index             = 0
state                     = "SEARCH"
last_seen_x               = 160
last_seen_y               = 120
park_start                = None
search_start_time         = None
approach_start_time       = None
blind_dash_start_time     = None

last_dist_cm         = 150.0
boundary_phase       = 0
boundary_phase_start = None
boundary_direction   = 1

morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

frame_count = 0

# =========================================
# LIDAR 쓰레드 함수들
# =========================================
def _apply_ema(angle, dist_cm):
    if dist_cm <= 0:
        return
    _scan_buf[angle] = (1.0 - EMA_ALPHA) * _scan_buf[angle] + EMA_ALPHA * dist_cm

def _apply_median_filter():
    k = MEDIAN_K
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
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
        if ((raw[0] & 0x02) >> 1) != (1 - s_flag) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
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

# =========================================
# MOTOR COMM
# =========================================
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# IMAGE MASK
# =========================================
def make_mask(frame, hsv, color_name):
    cfg = COLOR_CFG[color_name]
    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    hsv_mask = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"] is not None:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        hsv_mask = cv2.bitwise_or(hsv_mask, cv2.inRange(hsv, lo2, hi2))
    bgr_mask = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))

    mask = cv2.bitwise_and(hsv_mask, bgr_mask)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morph_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_kernel)
    return mask

def flush_camera_buffer(n=8):
    for _ in range(n):
        cap.grab()

# =========================================
# BOUNDARY SEARCH
# =========================================
def start_boundary_search(lost_x, frame_cx):
    global boundary_phase, boundary_phase_start, boundary_direction, state
    boundary_direction = 1 if lost_x > frame_cx else -1
    boundary_phase = 1
    boundary_phase_start = time.time()
    state = "BOUNDARY"
    print(f"🔍 바운더리 탐색 시작 | 방향={'우' if boundary_direction > 0 else '좌'}")

def run_boundary_search():
    global boundary_phase, boundary_phase_start, state, search_start_time
    elapsed = time.time() - boundary_phase_start

    if boundary_phase == 1:
        if elapsed < BOUNDARY_PHASE1_SEC: return 0.03, BOUNDARY_W * boundary_direction
        boundary_phase = 2; boundary_phase_start = time.time(); elapsed = 0.0
    if boundary_phase == 2:
        if elapsed < BOUNDARY_PHASE2_SEC: return 0.03, BOUNDARY_W * -boundary_direction
        boundary_phase = 3; boundary_phase_start = time.time(); elapsed = 0.0
    if boundary_phase == 3:
        if elapsed < BOUNDARY_PHASE3_SEC: return BOUNDARY_BACK_V, 0.0
        boundary_phase = 0; state = "SEARCH"; search_start_time = time.time()
    return 0.0, 0.0

# =========================================
# 정렬 비율 계산 헬퍼  ← 신규 추가
# =========================================
def calc_align_ratio(error_x):
    """
    error_x가 ALIGN_THRESHOLD 이내면 1.0(풀 전진),
    이상이면 0.0(전진 없음)으로 선형 보간.
    """
    return max(0.0, 1.0 - abs(error_x) / ALIGN_THRESHOLD)

# =========================================
# MAIN LOOP
# =========================================
print("🏁 MISSION CONTROL v7.5 Active (Centroid-Align Drive)")

try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame_count += 1
        frame = cv2.flip(frame, 1)
        HEIGHT, WIDTH = frame.shape[:2]
        frame_cx = WIDTH // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if mission_index >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSION COMPLETE!", (20, HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        target = MISSION[mission_index]
        draw   = COLOR_CFG[target]["draw"]
        mask   = make_mask(frame, hsv, target)

        contours_full, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        with scan_lock:
            front_angles = list(range(345, 360)) + list(range(0, 15))
            front_dists = [_scan_shared[a] for a in front_angles if 3.0 < _scan_shared[a] < 150.0]
            min_front_lidar = min(front_dists) if front_dists else float('inf')

        valid_contours = []
        for c in contours_full:
            area = cv2.contourArea(c)
            if area < MIN_AREA:
                continue

            rect = cv2.minAreaRect(c)
            (cx, cy_obj), (w_box, h_box), angle = rect

            if cy_obj < 55:
                continue

            bottom_y = int(cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3])
            bottom_y = min(bottom_y, RES_H - 1)
            est_dist_cm = pixel_to_ground_dist(bottom_y)

            if target == "blue":
                if min_front_lidar != float('inf') and abs(est_dist_cm - min_front_lidar) < 22.0:
                    continue
                if h_box > w_box * 1.3:
                    continue

            valid_contours.append((c, area, rect, est_dist_cm, bottom_y))

        object_detected = len(valid_contours) > 0

        if object_detected:
            c_main, area, rect, est_dist_cm, bottom_y = max(valid_contours, key=lambda x: x[1])
            (cx, cy_obj), _, _ = rect
            cx, cy_obj = int(cx), int(cy_obj)

            if state == "FORCED_SEARCH":
                print(f"🎯 [타겟 포착] 회전 중 {target} 발견! 관성 제어를 위해 순간 정지.")
                stop_robot()
                time.sleep(0.4)
                flush_camera_buffer(n=6)

            last_seen_x = int(0.6 * last_seen_x + 0.4 * cx)
            error_x = last_seen_x - frame_cx
            last_seen_y = bottom_y

            if est_dist_cm != float('inf'):
                last_dist_cm = est_dist_cm

            cv2.drawContours(frame, [np.int32(cv2.boxPoints(rect))], 0, draw, 2)

            # 정렬 상태 시각화: 센트로이드와 카메라 중심 사이 선
            cv2.line(frame, (frame_cx, cy_obj), (cx, cy_obj), (0, 255, 255), 1)
            cv2.circle(frame, (cx, cy_obj), 5, draw, -1)
            cv2.circle(frame, (frame_cx, cy_obj), 4, (0, 255, 255), 1)

            if frame_count % 15 == 0:
                cx_clamped = np.clip(cx, 0, WIDTH - 1)
                cy_clamped = np.clip(cy_obj, 0, HEIGHT - 1)
                bgr_val = frame[cy_clamped, cx_clamped]
                hsv_val = hsv[cy_clamped, cx_clamped]
                align_r = calc_align_ratio(error_x)
                print(f"📊 Tracking [{target}] | Area: {int(area)} | Dist: {est_dist_cm:.1f}cm | AlignRatio: {align_r:.2f} | BGR: {bgr_val} | HSV: {hsv_val}")

            if state in ["BOUNDARY", "FORCED_SEARCH", "WANDERING", "SEARCH"]:
                boundary_phase = 0
                state = "TRACK"

        else:
            area = 0
            error_x = last_seen_x - frame_cx

        # ── 1. PARKING 상태 ───────────────────
        if state == "PARKING":
            send_cmd(0.0, 0.0)
            park_elapsed = time.time() - park_start

            cv2.putText(frame, f"STATE: PARKING ({PARK_SEC - park_elapsed:.1f}s)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("frame", frame)

            if park_elapsed >= PARK_SEC:
                print(f"✅ [{target}] 안착 완료 -> 다음 미션 전환")
                arduino_ser.write(b"RESET\n")

                mission_index += 1
                blind_dash_start_time = None
                boundary_phase = 0
                last_seen_y = 120

                if mission_index < len(MISSION):
                    flush_camera_buffer(n=15)
                    state = "FORCED_SEARCH"
                    search_start_time = time.time()
                    print(f"🔄 다음 미션 [{MISSION[mission_index]}] 시작! 제자리 회전 탐색(FORCED_SEARCH) 돌입")

            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        # ── 2. APPROACH 상태 타임아웃 ────────────────────────────
        if state == "APPROACH" and approach_start_time is not None:
            if time.time() - approach_start_time > APPROACH_MAX_TIMEOUT:
                state = "APPROACH_BLIND"
                blind_dash_start_time = time.time()

        # ── 3. 주행 로직 및 조건 판단 ────────────────────────
        cam_v, cam_w = 0.0, 0.0
        cam_state = state

        if state == "APPROACH_BLIND":
            cam_v = BLIND_V
            cam_w = 0.0
            cam_state = "APPROACH_BLIND"

            time_condition  = (time.time() - blind_dash_start_time >= APPROACH_DRIVE_SEC)
            clear_condition = (not object_detected)

            if time_condition and clear_condition:
                state = "PARKING"
                park_start = time.time()
                print("🏁 [조건 충족] 강제 전진 완료 -> 정차(PARKING) 진입")
            else:
                if not time_condition:
                    cv2.putText(frame, f"DASHING: {time.time()-blind_dash_start_time:.1f}s / {APPROACH_DRIVE_SEC}s", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                if not clear_condition:
                    cv2.putText(frame, "WAITING: OBJ STILL VISIBLE", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        else:
            if object_detected:
                if state == "APPROACH" or area > TARGET_AREA or last_seen_y >= 200:
                    if state != "APPROACH":
                        state = "APPROACH"
                        approach_start_time = time.time()

                    if last_seen_y >= 215:
                        state = "APPROACH_BLIND"
                        blind_dash_start_time = time.time()
                        print("🚀 [하단 한계 진입] APPROACH_BLIND 시작")
                        cam_v, cam_w = BLIND_V, 0.0
                    else:
                        # ── APPROACH: 정렬 후 전진  ← 수정
                        align_ratio = calc_align_ratio(error_x)
                        cam_w = -KP_ROT * error_x * (1.0 if abs(error_x) > ALIGN_THRESHOLD else 0.5)
                        cam_v = APPROACH_V * align_ratio
                    cam_state = "APPROACH"

                else:
                    # ── TRACK: 정렬 후 전진  ← 수정
                    align_ratio = calc_align_ratio(error_x)

                    cam_w = -KP_ROT * error_x
                    if abs(error_x) > ALIGN_THRESHOLD:
                        cam_w *= ALIGN_SCALE  # 정렬 안 됐을 때 회전력 강화

                    rem_area = max(0.0, TARGET_AREA - area)
                    base_v   = MIN_V + (MAX_V - MIN_V) * (rem_area / TARGET_AREA)
                    cam_v    = np.clip(base_v * align_ratio, 0.0, MAX_V)
                    cam_state = "TRACK"

            else:
                if state == "APPROACH":
                    state = "APPROACH_BLIND"
                    blind_dash_start_time = time.time()
                    print("🚀 [접근 중 급작 유실] APPROACH_BLIND 즉시 시작")
                    cam_v, cam_w = BLIND_V, 0.0
                elif state == "BOUNDARY":
                    cam_v, cam_w = run_boundary_search()
                    cam_state = f"BOUNDARY-P{boundary_phase}"
                elif state == "FORCED_SEARCH":
                    if time.time() - search_start_time > SEARCH_TIMEOUT:
                        start_boundary_search(last_seen_x, frame_cx)
                    else:
                        cam_v = 0.02
                        cam_w = -1.35 if last_seen_x > frame_cx else 1.35
                elif state == "WANDERING":
                    cam_v, cam_w = 0.20, 0.0
                else:
                    cam_state = "SEARCH-L" if last_seen_x <= frame_cx else "SEARCH-R"
                    cam_v = 0.03
                    cam_w = -1.30 if last_seen_x > frame_cx else 1.30

        # ── 모터 명령 전송 ─────────────────────────────────────────
        send_cmd(cam_v, cam_w)

        # ── 디스플레이 ────────────────────────────────────────────
        align_ratio_disp = calc_align_ratio(error_x) if object_detected else 0.0
        cv2.putText(frame, f"STATE: {cam_state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"TARGET: {target}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
        cv2.putText(frame, f"ALIGN: {align_ratio_disp:.2f} | ERR: {error_x:+d}px", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"LIDAR FRONT: {min_front_lidar:.1f} cm", (20, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("USER INTERRUPT")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
