import cv2
import serial
import numpy as np
import time
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
# CAMERA PARAMETERS (V55 광각 기준)
# =========================================
MAX_V       = 0.24
MIN_V       = 0.10
KP_ROT      = 0.003
MIN_AREA    = 400        # 광각 대응 하한
TARGET_AREA = 6000       # APPROACH 진입 면적
PARK_SEC    = 3.0
APPROACH_V           = 0.22
APPROACH_MAX_TIMEOUT = 4.0
APPROACH_DRIVE_SEC   = 1.5
SEARCH_TIMEOUT       = 2.2

# =========================================
# [7조] ROI 도착 판정 파라미터
# =========================================
# 화면 하단 ROI 영역에 색 픽셀이 N프레임 연속 유지되면 도착으로 판정
# PARK_AREA(면적 임계값) 캘리브레이션 완전 제거
# 광각 FOV·조명 변화에 무관하게 안정적으로 동작
ROI_Y             = 180  # 하단 ROI 시작 y좌표 (240px 중 하단 60px 사용)
ROI_MIN_PIXEL     = 800  # ROI 내 최소 색 픽셀 수
ROI_CONFIRM_FRAMES = 8   # 연속 히트 프레임 수

# =========================================
# [1조] 바운더리 탐색 파라미터
# =========================================
# 마지막 인식 위치(last_seen_x) 기준으로 3단계 탐색
# 기존 단순 좌/우 회전 SEARCH보다 복구 성공률이 높음
BOUNDARY_PHASE1_SEC = 1.0   # 1단계: 유실 방향 호 탐색
BOUNDARY_PHASE2_SEC = 1.8   # 2단계: 반대 방향 확장 탐색
BOUNDARY_PHASE3_SEC = 0.8   # 3단계: 후진
BOUNDARY_W          = 0.55  # 탐색 회전 속도
BOUNDARY_BACK_V     = -0.10 # 3단계 후진 속도

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
last_seen_x          = 160     # [1조] 마지막 인식 x좌표 (탐색 기준점)
park_start           = None
search_start_time    = None
approach_start_time  = None
roi_count            = 0       # [7조] ROI 연속 히트 카운터

# [1조] 바운더리 탐색 내부 상태
boundary_phase       = 0       # 0=비활성 1=유실방향호 2=반대방향호 3=후진
boundary_phase_start = None
boundary_direction   = 1       # +1=우회전 -1=좌회전

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
    v = np.clip(v, -0.3, 0.3)
    w = np.clip(w, -0.8, 0.8)
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
# [1조] 바운더리 탐색 함수
# =========================================
def start_boundary_search(lost_x, frame_cx):
    """
    객체 유실 시 호출.
    last_seen_x 기반으로 방향을 결정하고 3단계 탐색을 시작.
      1단계: 객체가 이동했을 방향으로 호 탐색
      2단계: 반대 방향으로 더 넓게 탐색
      3단계: 후진 후 재시도 기회 부여
    """
    global boundary_phase, boundary_phase_start, boundary_direction, state
    # 유실 위치가 오른쪽이면 객체가 오른쪽으로 이동 → 우회전 탐색
    boundary_direction = 1 if lost_x > frame_cx else -1
    boundary_phase = 1
    boundary_phase_start = time.time()
    state = "BOUNDARY"
    dir_str = "우" if boundary_direction > 0 else "좌"
    print(f"🔍 [1조] 바운더리 탐색 시작 | 유실x={lost_x} | 1단계={dir_str}")

def run_boundary_search():
    """
    BOUNDARY 상태에서 매 프레임 호출하여 (v, w) 반환.
    3단계 모두 실패 시 기본 SEARCH로 복귀.
    """
    global boundary_phase, boundary_phase_start, state, search_start_time

    elapsed = time.time() - boundary_phase_start

    if boundary_phase == 1:
        if elapsed < BOUNDARY_PHASE1_SEC:
            return 0.03, BOUNDARY_W * boundary_direction
        boundary_phase = 2
        boundary_phase_start = time.time()
        print("🔍 [1조] 2단계: 반대 방향 확장 탐색")
        elapsed = 0.0

    if boundary_phase == 2:
        if elapsed < BOUNDARY_PHASE2_SEC:
            return 0.03, BOUNDARY_W * -boundary_direction
        boundary_phase = 3
        boundary_phase_start = time.time()
        print("🔍 [1조] 3단계: 후진")
        elapsed = 0.0

    if boundary_phase == 3:
        if elapsed < BOUNDARY_PHASE3_SEC:
            return BOUNDARY_BACK_V, 0.0
        # 3단계도 실패 → SEARCH 복귀
        boundary_phase = 0
        state = "SEARCH"
        search_start_time = time.time()
        print("⚠️ [1조] 바운더리 실패 → SEARCH 복귀")
        return 0.0, 0.0

    return 0.0, 0.0

# =========================================
# MAIN LOOP
# =========================================
print("🏁 MISSION CONTROL v2 (7조 ROI 도착판정 + 1조 바운더리 탐색)")

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

        # ── [PARKING 격리 레이어] ─────────────────────────────────
        if state == "PARKING":
            send_cmd(0.0, 0.0)
            elapsed = time.time() - park_start
            remain  = max(0.0, PARK_SEC - elapsed)

            cv2.putText(frame, "STATE: PARKING", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"WAIT [{target}]: {remain:.1f}s", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
            cv2.imshow("frame", frame)

            if elapsed >= PARK_SEC:
                print(f"✅ [{target}] 주차 완료 → 다음 미션")
                mission_index += 1
                roi_count = 0
                boundary_phase = 0

                if mission_index < len(MISSION):
                    flush_camera_buffer(n=15)
                    # 다음 타겟은 현재 위치 기준 바운더리 탐색으로 시작
                    start_boundary_search(last_seen_x, frame_cx)
                    print(f"➡️  NEXT TARGET: [{MISSION[mission_index]}] 바운더리 탐색 시작")
                else:
                    print("🏁 모든 미션 클리어")

            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        # ── APPROACH 타임아웃 안전장치 ────────────────────────────
        if state == "APPROACH" and approach_start_time is not None:
            if time.time() - approach_start_time > APPROACH_MAX_TIMEOUT:
                print(f"🚨 [안전장치] APPROACH {APPROACH_MAX_TIMEOUT}초 만료 → PARKING")
                state      = "PARKING"
                park_start = time.time()
                roi_count  = 0
                continue

        # ── 마스크 & 컨투어 ───────────────────────────────────────
        mask = make_mask(frame, hsv, target)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ── [7조] ROI 도착 판정 ───────────────────────────────────
        roi_mask   = mask[ROI_Y:, :]
        roi_pixels = cv2.countNonZero(roi_mask)
        roi_hit    = (roi_pixels >= ROI_MIN_PIXEL)

        # APPROACH 중에만 카운트 (TRACK 중 오인 방지)
        if roi_hit and state == "APPROACH":
            roi_count += 1
        else:
            roi_count = 0

        # N프레임 연속 히트 → 도착
        if roi_count >= ROI_CONFIRM_FRAMES:
            state      = "PARKING"
            park_start = time.time()
            roi_count  = 0
            print(f"🅿️  [{target}] [7조] ROI {ROI_CONFIRM_FRAMES}f 확정 → PARKING "
                  f"(roi_px={roi_pixels})")
            continue

        # ROI 라인 시각화
        cv2.line(frame, (0, ROI_Y), (WIDTH, ROI_Y), (0, 255, 0), 1)
        cv2.putText(frame, f"ROI:{roi_pixels}px {roi_count}f",
                    (5, ROI_Y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # ── 주행 연산 ─────────────────────────────────────────────
        cam_v, cam_w = 0.0, 0.0
        cam_state = state

        # ── [객체 인지] ───────────────────────────────────────────
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                # 객체 발견 → 바운더리/탐색 해제
                if state in ["BOUNDARY", "FORCED_SEARCH", "WANDERING", "SEARCH"]:
                    boundary_phase = 0
                    state = "TRACK"
                    print(f"🎯 [{target}] 포착 → TRACK (area={int(area)})")

                rect = cv2.minAreaRect(c)
                (cx, cy), _, _ = rect
                cx, cy = int(cx), int(cy)
                last_seen_x = cx  # [1조] 마지막 인식 위치 갱신

                cv2.drawContours(frame, [np.int32(cv2.boxPoints(rect))], 0, draw, 2)
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                error_x = cx - frame_cx

                # 캘리브레이션 로그 (TRACK 상태)
                if state == "TRACK":
                    with scan_lock:
                        lidar_front = float(_scan_shared[0])
                    print(f"[CAL] lidar={lidar_front:.1f}cm  area={int(area):6d}")

                # APPROACH 진입
                if state == "APPROACH" or area > TARGET_AREA:
                    if state != "APPROACH":
                        state = "APPROACH"
                        approach_start_time = time.time()
                        print(f"📥 [{target}] APPROACH 진입 (area={int(area)})")

                    elapsed_ap = time.time() - approach_start_time
                    print(f"[APPROACH] area={int(area):6d}  roi={roi_pixels}px  "
                          f"roi_count={roi_count}  elapsed={elapsed_ap:.2f}s")

                    cam_w = -KP_ROT * error_x * 0.5
                    cam_v = APPROACH_V
                    cam_state = "APPROACH"

                else:
                    # TRACK: 면적 비례 속도
                    cam_w    = -KP_ROT * error_x
                    rem_area = max(0.0, TARGET_AREA - area)
                    cam_v    = MIN_V + (MAX_V - MIN_V) * (rem_area / TARGET_AREA)
                    cam_v    = np.clip(cam_v, MIN_V, MAX_V)
                    cam_state = "TRACK"

                cv2.putText(frame, f"Area:{int(area)}", (20, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            else:
                # 면적 미달(노이즈) → 바운더리 시작
                if state not in ["BOUNDARY", "FORCED_SEARCH", "WANDERING", "PARKING"]:
                    start_boundary_search(last_seen_x, frame_cx)

        # ── [객체 유실] ───────────────────────────────────────────
        else:
            if state == "APPROACH":
                # 사각지대 블라인드 직진
                cam_v, cam_w = APPROACH_V, 0.0
                cam_state    = "APPROACH_BLIND"
                if time.time() - approach_start_time > APPROACH_DRIVE_SEC:
                    state      = "PARKING"
                    park_start = time.time()
                    roi_count  = 0
                    print(f"🅿️  [{target}] 블라인드 {APPROACH_DRIVE_SEC}s 완료 → PARKING")

            elif state == "BOUNDARY":
                # [1조] 바운더리 탐색 실행
                cam_v, cam_w = run_boundary_search()
                cam_state = f"BOUNDARY-P{boundary_phase}"

            elif state == "FORCED_SEARCH":
                if time.time() - search_start_time > SEARCH_TIMEOUT:
                    start_boundary_search(last_seen_x, frame_cx)
                else:
                    cam_v, cam_w = 0.03, 0.80

            elif state == "WANDERING":
                cam_v, cam_w = 0.20, 0.0

            else:
                # 기본 SEARCH: last_seen_x 방향 회전
                cam_state = "SEARCH-L" if last_seen_x <= frame_cx else "SEARCH-R"
                cam_v = 0.03
                cam_w = -0.65 if last_seen_x > frame_cx else 0.65

        # ── 모터 명령 ─────────────────────────────────────────────
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
