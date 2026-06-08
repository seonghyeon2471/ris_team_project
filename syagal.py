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
# [2조] 카메라 기하학 파라미터
# =========================================
CAM_H        = 73.0   # cm, 지면 기준 카메라 높이
CAM_PITCH    = 30.0   # deg, 수평 기준 하향각 (양수 = 아래 방향)
CAM_FOV_V    = 50.0   # deg, V55 수직 FOV
RES_W, RES_H = 320, 240
CY           = RES_H / 2   # 광축 y픽셀 (120px)
FY           = (RES_H / 2) / math.tan(math.radians(CAM_FOV_V / 2))  # ≈ 257px

def pixel_to_ground_dist(y_px):
    delta_deg  = math.degrees(math.atan2(y_px - CY, FY))
    total_deg  = CAM_PITCH + delta_deg
    if total_deg <= 0:
        return float('inf')
    return CAM_H / math.tan(math.radians(total_deg))

# =========================================
# CAMERA PARAMETERS (V55 광각 기준)
# =========================================
MAX_V       = 0.24
MIN_V       = 0.10
KP_ROT      = 0.003
MIN_AREA    = 400        # 광각 대응 하한
TARGET_AREA = 6000       # APPROACH 진입 면적

# 🌟 [속도 폭발 및 시간 최적화] 전진 대시 및 회전수색 파라미터 과감한 상향
APPROACH_V           = 0.24  # 정렬 주행 속도 소폭 상향
BLIND_V              = 0.29  # 사각지대 강제 전진 속도 대폭 상향 (기존 0.25 -> 한계점 돌입)
APPROACH_DRIVE_SEC   = 0.8   # 돌진 속도가 빨라진 만큼 돌진 시간 압축 (기존 1.2초 -> 0.8초로 지나침 방지)
APPROACH_MAX_TIMEOUT = 3.5   # 접근 전체 타임아웃 소폭 압축
SEARCH_TIMEOUT       = 1.6   # 회전 속도가 빨라졌으므로 360도 수색 타임아웃 단축 (기존 2.2초 -> 1.6초)

# =========================================
# [7조] ROI 도착 판정 파라미터
# =========================================
ROI_Y              = 180   # 하단 ROI 시작 y
ROI_MIN_PIXEL      = 800   # ROI 내 최소 색 픽셀
ROI_CONFIRM_FRAMES = 8    # 연속 N프레임 히트 → 도착 확정

# =========================================
# 색지 내부 정지 조건
# =========================================
INSIDE_STOP_SEC = 1.0     # 색지 내부 정지 유지 시간 (초)
PARK_SEC        = 3.0     # 전체 PARKING 대기 시간

# =========================================
# [1조] 바운더리 탐색 파라미터
# =========================================
BOUNDARY_PHASE1_SEC = 1.0
BOUNDARY_PHASE2_SEC = 1.8
BOUNDARY_PHASE3_SEC = 0.8
BOUNDARY_W          = 0.55
BOUNDARY_BACK_V     = -0.10

# =========================================
# 색상별 HSV 범위
# =========================================
COLOR_CFG = {
    "red": {
        "hsv1": ([0,   45,  50], [15,  255, 255]),
        "hsv2": ([160, 45,  50], [179, 255, 255]),
        "bgr":  ([0, 0, 0], [255, 255, 255]),
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv1": ([15,  30,  60], [40,  255, 255]),
        "hsv2": ([10,  0,  190], [45,  45,  255]),
        "bgr":  ([0, 0, 0], [255, 255, 255]),
        "draw": (0, 200, 255),
    },
    "blue": {
        "hsv1": ([90,  45,  40], [140, 255, 255]),
        "hsv2": None,
        "bgr":  ([0, 0, 0], [255, 255, 255]),
        "draw": (255, 80, 0),
    },
}

MISSION = ["red", "yellow", "blue"]

# =========================================
# 상태 변수
# =========================================
mission_index        = 0
state                = "SEARCH"
last_seen_x          = 160
last_seen_y          = 120    
park_start           = None
inside_stop_start    = None
search_start_time    = None
approach_start_time  = None
roi_count            = 0

# 근거리 유실 판단 변수
last_dist_cm         = 150.0  
NEAR_DIST_THRESHOLD  = 50.0   

# [1조] 바운더리 탐색 내부 상태
boundary_phase       = 0
boundary_phase_start = None
boundary_direction   = 1

# =========================================
# LIDAR
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
        if ((raw[0] & 0x02) >> 1) != (1 - s_flag) \
                or (raw[1] & 0x01) != 1 \
                or (raw[0] >> 2) < 3:
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
# MOTOR
# =========================================
def send_cmd(v, w):
    # 🌟 [속도 폭발 및 시간 최적화] 하드웨어 출력이 잘리지 않도록 클립 범위 확장 (v: 0.40, w: 1.60)
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# MASK
# =========================================
def make_mask(frame, hsv, color_name):
    cfg = COLOR_CFG[color_name]
    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    hsv_mask = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"] is not None:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        hsv_mask = cv2.bitwise_or(hsv_mask, cv2.inRange(hsv, lo2, hi2))
    bgr_mask = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    return cv2.bitwise_and(hsv_mask, bgr_mask)

def flush_camera_buffer(n=8):
    for _ in range(n):
        cap.grab()

# =========================================
# [1조] 바운더리 탐색
# =========================================
def start_boundary_search(lost_x, frame_cx):
    global boundary_phase, boundary_phase_start, boundary_direction, state
    boundary_direction = 1 if lost_x > frame_cx else -1
    boundary_phase = 1
    boundary_phase_start = time.time()
    state = "BOUNDARY"
    print(f"🔍 [1조] 바운더리 탐색 | 유실x={lost_x} | "
          f"1단계={'우' if boundary_direction > 0 else '좌'}")

def run_boundary_search():
    global boundary_phase, boundary_phase_start, state, search_start_time
    elapsed = time.time() - boundary_phase_start

    if boundary_phase == 1:
        if elapsed < BOUNDARY_PHASE1_SEC:
            return 0.03, BOUNDARY_W * boundary_direction
        boundary_phase = 2
        boundary_phase_start = time.time()
        elapsed = 0.0
        print("🔍 [1조] 2단계: 반대 방향 확장 탐색")

    if boundary_phase == 2:
        if elapsed < BOUNDARY_PHASE2_SEC:
            return 0.03, BOUNDARY_W * -boundary_direction
        boundary_phase = 3
        boundary_phase_start = time.time()
        elapsed = 0.0
        print("🔍 [1조] 3단계: 후진")

    if boundary_phase == 3:
        if elapsed < BOUNDARY_PHASE3_SEC:
            return BOUNDARY_BACK_V, 0.0
        boundary_phase = 0
        state = "SEARCH"
        search_start_time = time.time()
        print("⚠️ [1조] 바운더리 실패 → SEARCH 복귀")
        return 0.0, 0.0

    return 0.0, 0.0

# =========================================
# MAIN LOOP
# =========================================
print("🏁 MISSION CONTROL v3 (Maximum Speed Profile Loaded)")
print(f"   카메라: h={CAM_H}cm  pitch={CAM_PITCH}°  fy={FY:.1f}px")
print(f"   ROI y={ROI_Y} → 지면 거리 {pixel_to_ground_dist(ROI_Y):.1f}cm 이내 = 색지 내부")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame    = cv2.flip(frame, 1)
        HEIGHT, WIDTH = frame.shape[:2]
        frame_cx = WIDTH // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ── 미션 완료 ─────────────────────────────────────────────
        if mission_index >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSION COMPLETE!", (20, HEIGHT // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        target = MISSION[mission_index]
        draw   = COLOR_CFG[target]["draw"]

        # ── 마스크 ────────────────────────────────────────────────
        mask = make_mask(frame, hsv, target)

        # ── [7조 + 2조] ROI 및 거리 계산 ─────────────────────────
        roi_mask   = mask[ROI_Y:, :]
        roi_pixels = cv2.countNonZero(roi_mask)
        roi_hit    = (roi_pixels >= ROI_MIN_PIXEL)

        contours_full, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)
        est_dist_cm = float('inf')
        if contours_full:
            c_main = max(contours_full, key=cv2.contourArea)
            if cv2.contourArea(c_main) > MIN_AREA:
                bottom_y = int(cv2.boundingRect(c_main)[1] +
                               cv2.boundingRect(c_main)[3])
                bottom_y = min(bottom_y, RES_H - 1)
                est_dist_cm = pixel_to_ground_dist(bottom_y)
                
                last_seen_y = bottom_y
                
                if est_dist_cm != float('inf'):
                    last_dist_cm = est_dist_cm

        # ── [PARKING 격리 레이어] ─────────────────────────────────
        if state == "PARKING":
            send_cmd(0.0, 0.0)

            if roi_hit:
                if inside_stop_start is None:
                    inside_stop_start = time.time()
                    print(f"🟢 [{target}] 색지 내부 정지 시작")
                inside_elapsed = time.time() - inside_stop_start
            else:
                if inside_stop_start is not None:
                    print(f"⚠️  [{target}] ROI 유실 → 내부 정지 타이머 리셋")
                inside_stop_start = None
                inside_elapsed = 0.0

            park_elapsed = time.time() - park_start
            remain_park  = max(0.0, PARK_SEC - park_elapsed)
            remain_inside = max(0.0, INSIDE_STOP_SEC -
                                (inside_elapsed if inside_stop_start else 0.0))

            cv2.putText(frame, "STATE: PARKING", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"TARGET: {target}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
            cv2.putText(frame, f"PARK: {remain_park:.1f}s", (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            color_inside = (0, 255, 0) if roi_hit else (0, 100, 100)
            cv2.putText(frame, f"INSIDE: {remain_inside:.1f}s  ROI:{roi_pixels}px",
                        (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_inside, 1)
            cv2.line(frame, (0, ROI_Y), (WIDTH, ROI_Y), color_inside, 1)
            cv2.imshow("frame", frame)
            cv2.imshow("mask",  mask)

            inside_ok = (inside_stop_start is not None and
                         (time.time() - inside_stop_start) >= INSIDE_STOP_SEC)
            if park_elapsed >= PARK_SEC and inside_ok:
                print(f"✅ [{target}] 내부 정지 {INSIDE_STOP_SEC}s 완료 → 다음 미션")
                mission_index += 1
                roi_count = 0
                inside_stop_start = None
                boundary_phase = 0
                last_dist_cm = 150.0  
                last_seen_y = 120    

                if mission_index < len(MISSION):
                    flush_camera_buffer(n=15)
                    start_boundary_search(last_seen_x, frame_cx)
                    print(f"➡️  NEXT TARGET: [{MISSION[mission_index]}]")
                else:
                    print("🏁 모든 미션 클리어")

            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        # ── APPROACH 타임아웃 안전장치 ────────────────────────────
        if state == "APPROACH" and approach_start_time is not None:
            if time.time() - approach_start_time > APPROACH_MAX_TIMEOUT:
                print(f"🚨 [안전장치] APPROACH {APPROACH_MAX_TIMEOUT}s 만료 → PARKING")
                state             = "PARKING"
                park_start        = time.time()
                inside_stop_start = None
                roi_count         = 0
                continue

        # ── [7조] ROI 도착 판정 (APPROACH 중) ────────────────────
        if state == "APPROACH":
            if roi_hit:
                roi_count += 1
            else:
                roi_count = 0

            if roi_count >= ROI_CONFIRM_FRAMES:
                state             = "PARKING"
                park_start        = time.time()
                inside_stop_start = None
                roi_count         = 0
                print(f"🅿️  [{target}] [7조] ROI {ROI_CONFIRM_FRAMES}f 확정 "
                      f"(dist≈{est_dist_cm:.1f}cm) → PARKING")
                continue

        # ── 컨투어 ────────────────────────────────────────────────
        contours = contours_full

        roi_color = (0, 255, 0) if roi_hit else (100, 100, 100)
        cv2.line(frame, (0, ROI_Y), (WIDTH, ROI_Y), roi_color, 1)
        cv2.putText(frame, f"ROI:{roi_pixels}px {roi_count}f  dist:{est_dist_cm:.0f}cm",
                    (5, ROI_Y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, roi_color, 1)

        cam_v, cam_w = 0.0, 0.0
        cam_state = state

        # ── [객체 인지] ───────────────────────────────────────────
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                if state in ["BOUNDARY", "FORCED_SEARCH", "WANDERING", "SEARCH"]:
                    boundary_phase = 0
                    state = "TRACK"
                    print(f"🎯 [{target}] 포착 → TRACK (area={int(area)} | dist≈{est_dist_cm:.1f}cm)")

                rect = cv2.minAreaRect(c)
                (cx, cy_obj), _, _ = rect
                cx, cy_obj = int(cx), int(cy_obj)
                last_seen_x = cx

                cv2.drawContours(frame, [np.int32(cv2.boxPoints(rect))], 0, draw, 2)
                cv2.circle(frame, (cx, cy_obj), 5, (255, 0, 0), -1)
                error_x = cx - frame_cx

                if state == "TRACK":
                    pass

                if state == "APPROACH" or area > TARGET_AREA:
                    if state != "APPROACH":
                        state = "APPROACH"
                        approach_start_time = time.time()
                        roi_count = 0
                        print(f"📥 [{target}] APPROACH 진입 (area={int(area)} | dist≈{est_dist_cm:.1f}cm)")

                    cam_w = -KP_ROT * error_x * 0.5
                    cam_v = APPROACH_V
                    cam_state = "APPROACH"

                else:
                    cam_w    = -KP_ROT * error_x
                    rem_area = max(0.0, TARGET_AREA - area)
                    cam_v    = MIN_V + (MAX_V - MIN_V) * (rem_area / TARGET_AREA)
                    cam_v    = np.clip(cam_v, MIN_V, MAX_V)
                    cam_state = "TRACK"

                cv2.putText(frame, f"Area:{int(area)}  {est_dist_cm:.0f}cm",
                            (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            else:
                if state not in ["BOUNDARY", "FORCED_SEARCH", "WANDERING", "PARKING", "APPROACH"]:
                    if last_dist_cm <= NEAR_DIST_THRESHOLD:
                        if last_seen_y >= 210:
                            state = "APPROACH"
                            approach_start_time = time.time()
                            print(f"🚀 [하단 유실 예측] Y={last_seen_y} >= 210 -> 강제 직진(BLIND) 후 주차 연계")
                        else:
                            state = "SEARCH"
                            search_start_time = time.time()
                            print(f"🔄 [근거리 좌우유실] 최종거리 {last_dist_cm:.1f}cm -> 회전 탐색(SEARCH) 전환")
                    else:
                        start_boundary_search(last_seen_x, frame_cx)

        # ── [객체 유실 - 완전히 잡히지 않을 때] ───────────────────
        else:
            if state == "APPROACH":
                # 🌟 [속도 폭발 및 시간 최적화] 사각지대 전진 돌진 속도 최고조 상향 (BLIND_V = 0.29)
                cam_v, cam_w = BLIND_V, 0.0
                cam_state    = "APPROACH_BLIND"
                if time.time() - approach_start_time > APPROACH_DRIVE_SEC:
                    state             = "PARKING"
                    park_start        = time.time()
                    inside_stop_start = None
                    roi_count         = 0
                    print(f"🅿️  [{target}] 블라인드 완료 → PARKING")

            elif state == "BOUNDARY":
                cam_v, cam_w = run_boundary_search()
                cam_state = f"BOUNDARY-P{boundary_phase}"

            elif state == "FORCED_SEARCH":
                if time.time() - search_start_time > SEARCH_TIMEOUT:
                    start_boundary_search(last_seen_x, frame_cx)
                else:
                    # 🌟 [속도 폭발 및 시간 최적화] 강제 회전 수색 각속도 대폭 상향 (기존 1.05 -> 1.35)
                    cam_v, cam_w = 0.03, 1.35

            elif state == "WANDERING":
                cam_v, cam_w = 0.20, 0.0

            else:
                if last_dist_cm <= NEAR_DIST_THRESHOLD and last_seen_y >= 210:
                    state = "APPROACH"
                    approach_start_time = time.time()
                    print(f"🚀 [하단 완전유실] Y={last_seen_y} -> 강제 직진(BLIND) 상태 트리거")
                else:
                    cam_state = "SEARCH-L" if last_seen_x <= frame_cx else "SEARCH-R"
                    cam_v = 0.03
                    # 🌟 [속도 폭발 및 시간 최적화] 일반 제자리 회전 탐색 각속도 대폭 상향 (기존 1.00 -> 1.30)
                    cam_w = -1.30 if last_seen_x > frame_cx else 1.30

        # ── 모터 전달 ─────────────────────────────────────────────
        send_cmd(cam_v, cam_w)

        # ── 디스플레이 ────────────────────────────────────────────
        cv2.putText(frame, f"STATE: {cam_state}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"TARGET: {target}", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)

        cv2.imshow("frame", frame)
        cv2.imshow("mask",  mask)

        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("USER INTERRUPT")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
    print("SYSTEM SHUTDOWN")
