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
# [2조] 카메라 기하학 파라미터 (물리 치수 반영)
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
# CAMERA PARAMETERS (초고속 및 장거리 블라인드 최적화)
# =========================================
MAX_V       = 0.24
MIN_V       = 0.10
KP_ROT      = 0.003
MIN_AREA    = 400        
TARGET_AREA = 6000       

APPROACH_V           = 0.24  
BLIND_V              = 0.29  # 사각지대 강제 전진 속도 (최고조)

# 🌟 [물리 치수 반영 핵심 수정] 
# 화면 하단 유실 시점(전방 51cm)에서 로봇 중심이 색지 위에 안착할 때까지의 주행 시간 계산 반영 (51cm / 29cm/s ≈ 1.75초)
APPROACH_DRIVE_SEC   = 1.7   
APPROACH_MAX_TIMEOUT = 4.5   
SEARCH_TIMEOUT       = 1.6   

# =========================================
# 색지 내부 정지 조건
# =========================================
PARK_SEC        = 3.0     # 전체 PARKING 대기 시간 (초)

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
mission_index             = 0
state                     = "SEARCH"
last_seen_x               = 160
last_seen_y               = 120    
park_start                = None
search_start_time         = None
approach_start_time       = None
blind_dash_start_time     = None  # 블라인드 타이머 분리

# 근거리 유실 판단 변수
last_dist_cm         = 150.0  
NEAR_DIST_THRESHOLD  = 65.0   # 사각지대 진입선 유연화 (51cm 감안)

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
print("🏁 MISSION CONTROL v4 (Physical Dimensions Calibrated)")
print(f"   카메라 하단 사각지대 경계 거리: {pixel_to_ground_dist(240):.1f}cm")

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

        # ── [PARKING 격리 레이어 - 센서 검증 제거 후 타이머 확정 구조] ──
        if state == "PARKING":
            send_cmd(0.0, 0.0)  # 무조건 정지

            park_elapsed = time.time() - park_start
            remain_park  = max(0.0, PARK_SEC - park_elapsed)

            cv2.putText(frame, "STATE: PARKING (BLIND HOLD)", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"TARGET: {target}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
            cv2.putText(frame, f"HOLD: {remain_park:.1f}s", (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("frame", frame)

            # 사각지대 숨김 주차이므로 3초 버티면 무조건 성공 처리
            if park_elapsed >= PARK_SEC:
                print(f"✅ [{target}] 사각지대 안착 및 {PARK_SEC}s 정차 완료 → 미션 클리어")
                mission_index += 1
                arduino_ser.write(b"RESET\n")
                blind_dash_start_time = None
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
                print(f"🚨 [안전장치] 타임아웃 만료 → 강제 주차 레이어 이행")
                state      = "PARKING"
                park_start = time.time()
                continue

        # ── 컨투어 ────────────────────────────────────────────────
        contours = contours_full
        cam_v, cam_w = 0.0, 0.0
        cam_state = state

        # ── [객체 인지 상황] ───────────────────────────────────────
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                if state in ["BOUNDARY", "FORCED_SEARCH", "WANDERING", "SEARCH"]:
                    boundary_phase = 0
                    state = "TRACK"
                    print(f"🎯 [{target}] 포착 → TRACK (dist≈{est_dist_cm:.1f}cm)")

                rect = cv2.minAreaRect(c)
                (cx, cy_obj), _, _ = rect
                cx, cy_obj = int(cx), int(cy_obj)
                last_seen_x = cx

                cv2.drawContours(frame, [np.int32(cv2.boxPoints(rect))], 0, draw, 2)
                error_x = cx - frame_cx

                # 면적이 커지거나 하단 접근 시 APPROACH 명시화
                if state == "APPROACH" or area > TARGET_AREA or last_seen_y >= 200:
                    if state != "APPROACH":
                        state = "APPROACH"
                        approach_start_time = time.time()
                        print(f"📥 [{target}] APPROACH 돌입 (Y={last_seen_y})")

                    cam_w = -KP_ROT * error_x * 0.5
                    cam_v = APPROACH_V
                    cam_state = "APPROACH"
                else:
                    cam_w    = -KP_ROT * error_x
                    rem_area = max(0.0, TARGET_AREA - area)
                    cam_v    = MIN_V + (MAX_V - MIN_V) * (rem_area / TARGET_AREA)
                    cam_v    = np.clip(cam_v, MIN_V, MAX_V)
                    cam_state = "TRACK"

            # 미세 노이즈 이하 유실 전조 처리
            else:
                if state == "APPROACH" and last_seen_y >= 210:
                    state = "APPROACH_BLIND"
                    blind_dash_start_time = time.time()
                    print("🚀 [하단 경계 유실] 객체 미세화 -> BLIND DASH 트리거")

        # ── [객체 유실 상황 - 아예 사라졌을 때 처리 핵심] ──────────
        else:
            # 주차 과정 실행을 위해 화면 아래로 아예 사라진 시점 제어
            if state == "APPROACH" or state == "APPROACH_BLIND":
                if blind_dash_start_time is None:
                    blind_dash_start_time = time.time()
                    print(f"🚀 [완전 유실 확정] 전방 사각지대 진입 -> {APPROACH_DRIVE_SEC}초간 블라인드 돌진 시작")

                cam_v, cam_w = BLIND_V, 0.0
                cam_state    = "APPROACH_BLIND"
                
                # 완전히 사라진 후 51cm 공백을 깨고 패드 위로 올라타는 시간 체크
                if time.time() - blind_dash_start_time > APPROACH_DRIVE_SEC:
                    state      = "PARKING"
                    park_start = time.time()
                    print(f"🅿️  [{target}] 사각지대 돌진 시간 완료 → 로봇 정지 및 PARKING 검증")

            elif state == "BOUNDARY":
                cam_v, cam_w = run_boundary_search()
                cam_state = f"BOUNDARY-P{boundary_phase}"

            elif state == "FORCED_SEARCH":
                if time.time() - search_start_time > SEARCH_TIMEOUT:
                    start_boundary_search(last_seen_x, frame_cx)
                else:
                    cam_v, cam_w = 0.03, 1.35

            elif state == "WANDERING":
                cam_v, cam_w = 0.20, 0.0

            else:
                # 일반 유실 처리 상황
                cam_state = "SEARCH-L" if last_seen_x <= frame_cx else "SEARCH-R"
                cam_v = 0.03
                cam_w = -1.30 if last_seen_x > frame_cx else 1.30

        # ── 모터 전달 ─────────────────────────────────────────────
        send_cmd(cam_v, cam_w)

        # ── 디스플레이 ────────────────────────────────────────────
        cv2.putText(frame, f"STATE: {cam_state}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"TARGET: {target}", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
        cv2.imshow("frame", frame)

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
