import cv2
import serial
import numpy as np
import time
import math
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

# ── CAMERA ────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR ─────────────────────────────────────────────────────────────
EMA_ALPHA    = 0.35
MEDIAN_K     = 2
FRONT_RANGE  = 90   
THRESH_SLOW  = 55.0
THRESH_TURN  = 30.0
THRESH_STOP  = 18.0

_scan     = np.full(360, 150.0, dtype=np.float32)
_scan_pub = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

def _ema(a, d):
    if d > 0:
        _scan[a] = (1 - EMA_ALPHA) * _scan[a] + EMA_ALPHA * d

def _median():
    k = MEDIAN_K
    buf = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        buf[i] = np.sort(_scan[idx])[k]
    _scan[:] = buf

def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150: _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock: _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock: return _scan_pub.copy()

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

# ★ 신규: 어느 쪽 벽을 추종할지 결정 (타겟이 보이면 타겟 쪽, 안 보이면 라이다 기준 더 넓게 트인 쪽)
# WALL_SEARCH -> WALL_APPROACH로 넘어가는 순간에만 호출, 추종 중에는 절대 다시 호출하지 않음
def decide_follow_side(adir, found, cx_obj, cx_mid):
    if found and cx_obj >= 0:
        return "L" if cx_obj < cx_mid else "R"
    return "L" if adir == 1 else "R"

# ★ 수정: L/R 각도 구간이 실제 좌/우와 반대로 매핑되어 있던 문제를 swap하여 수정
def side_dist(scan, side):
    if side == "L":
        idx = np.arange(265, 296) % 360   # (수정 전: 65, 96)
    else:
        idx = np.arange(65, 96) % 360     # (수정 전: 265, 296)
    return float(np.min(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ★ 신규: 320x240 카메라 기준 cx_obj(색지 중심 x좌표) -> LiDAR 각도(정면=0 기준 오프셋)로 변환
#   - 반환값은 절대각이 아니라 "정면 대비 오프셋"(도) 이며 음수/양수 모두 가능
#   - CAM_TO_LIDAR_SIGN 값으로 카메라-라이다 좌우 매핑을 보정 (실측 후 부호 확인 필요)
def cx_to_lidar_angle(cx_obj, cx_mid, frame_w=320):
    if cx_obj < 0:
        return 0.0
    ratio = (cx_obj - cx_mid) / (frame_w / 2.0)
    ratio = float(np.clip(ratio, -1.0, 1.0))
    return CAM_TO_LIDAR_SIGN * ratio * (CAMERA_HFOV / 2.0)

# ★ 신규: 색지 방향(target_angle, 정면 기준 오프셋) ±TARGET_CHECK_HALFWIDTH 범위의
#   LiDAR 최소 거리를 반환 -> 이 값이 작으면 색지 방향에 장애물이 있다는 뜻
def obstacle_on_target(scan, target_angle):
    center = int(round(target_angle)) % 360
    idx = np.arange(center - TARGET_CHECK_HALFWIDTH,
                     center + TARGET_CHECK_HALFWIDTH + 1) % 360
    return float(np.min(scan[idx]))

def wall_follow(scan, fm, adir, follow_side, target_bias_sign=0):
    sd          = side_dist(scan, follow_side)
    # ★ 수정: side_dist와 동일한 이유로 left_close/right_close 구간도 swap
    left_close  = side_min(scan, 240, 300)   # (수정 전: 60, 120)
    right_close = side_min(scan, 60, 120)    # (수정 전: 240, 300)
    sign = 1 if follow_side == "L" else -1

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1, False)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85, False)

    if left_close < SIDE_STOP:
        return (WALL_V * 0.7, -0.7, False)
    if right_close < SIDE_STOP:
        return (WALL_V * 0.7,  0.7, False)

    # ★ 신규: 병목 구간(양쪽 벽이 동시에 좁아짐) -> 한쪽 벽 추종 대신 중앙 정렬 제어로 전환
    if left_close < BOTTLENECK_ENTER and right_close < BOTTLENECK_ENTER:
        err_center = left_close - right_close   # 왼쪽이 더 가까우면 음수 -> 오른쪽으로(음의 w), 오른쪽이 더 가까우면 양수 -> 왼쪽으로(양의 w)
        w_center = float(np.clip(CENTER_KP * err_center, -0.9, 0.9))
        return (BOTTLENECK_V, w_center, False)

    # ★ 수정: 벽을 놓친 상태(lost=True)로 표시. 호출부에서 누적 회전각을 추적해
    #   한 바퀴(FULL_LOOP_THRESH) 돌아도 못 찾으면 WALL_SEARCH로 폴백시킴.
    #   방향은 기존과 동일하게 놓친 쪽(follow_side)과 같은 sign으로 제자리에 가깝게 회전.
    if sd > WALL_TARGET * 2.0:
        return (0.05, sign * WALL_LOST_W, True)

    err = sd - WALL_TARGET
    w   = sign * WALL_KP * err
    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V

    # ★ 신규: 마지막으로 본 색지 방향(target_bias_sign: -1=좌, +1=우, 0=없음)으로
    #   회전각(w)을 강하게 편향. 벽 추종 성분(w)보다 색지 편향 성분이 우세하도록
    #   TARGET_BIAS_WEIGHT 비율로 블렌딩 -> "벽은 충돌 방지용, 색지 방향을 적극 우선"
    if target_bias_sign != 0:
        w_bias_target = target_bias_sign * 0.9   # 편향 성분의 크기(최대 회전각 근처로 강하게)
        w = (1.0 - TARGET_BIAS_WEIGHT) * w + TARGET_BIAS_WEIGHT * w_bias_target

    w = float(np.clip(w, -0.9, 0.9))
    return (v, w, False)

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]), "hsv2": None, "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),    "hsv2": None, "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]),   "hsv2": None, "bgr":  ([40,  0,   0], [255, 220, 220]), "draw": (255, 80, 0)},
}
MISSION = ["red", "yellow", "blue"]

def make_mask(frame, hsv, name):
    cfg = COLOR_CFG[name]
    lo1, hi1 = np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])
    m = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"]:
        lo2, hi2 = np.array(cfg["hsv2"][0]), np.array(cfg["hsv2"][1])
        m = cv2.bitwise_or(m, cv2.inRange(hsv, lo2, hi2))
    bm = cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1]))
    return cv2.bitwise_and(m, bm)

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.030
W_MIN          = 0.20
APPROACH_V     = 0.13
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

WALL_TARGET    = 10.0    # (수정: 20.0 -> 10.0) 목표 이격거리
SIDE_STOP      = 6.0     # ★ 신규: 측면 "너무 가까움" 비상회피 기준 (WALL_TARGET보다 작아야 P제어가 정상 작동)
BOTTLENECK_ENTER = 25.0  # ★ 신규: 좌/우 모두 이 값보다 가까우면 "병목 구간"으로 판단
CENTER_KP      = 0.030   # ★ 신규: 병목 구간에서 중앙 정렬 제어 게인
BOTTLENECK_V   = 0.15    # ★ 신규: 병목 구간 통과 속도 (양쪽 벽 가까우므로 WALL_V보다 느리게)
WALL_SCAN_DIST = 150.0  
WALL_APPROACH_V = 0.20  
WALL_KP        = 0.024   # (수정: 0.012 -> 0.024) 목표거리가 절반이 되어 오차 범위도 절반이므로 게인을 2배로 보정
WALL_V         = 0.22
WALL_TURN_V    = 0.10
WALL_LOST_W    = 1.3     
WALL_SEARCH_W  = 1.1     
MISSION_TIMEOUT_SEC = 10.0  

# ★ 신규: 색지 방향 장애물 검사용 파라미터
TARGET_CLEAR_DIST      = 40.0   # 이 거리(cm)보다 가까우면 "막힘"으로 판단
TARGET_CHECK_HALFWIDTH = 15     # 색지 방향 기준 ±15도 검사
CAMERA_HFOV            = 60.0   # 카메라 수평 화각(deg) — 실측 후 보정 필요
CAM_TO_LIDAR_SIGN      = 1      # 좌우가 반대로 동작하면 -1로 변경

# ★ 신규: 벽 추종 중 "마지막으로 본 색지 방향"으로 회전각(w)을 편향시키는 강도
#   0.0 = 편향 없음(순수 벽 추종), 1.0 = 색지 방향 부호로 완전히 덮어씀
#   "강하게" 요청에 따라 벽 추종 성분보다 색지 편향 성분이 우세하도록 설정
TARGET_BIAS_WEIGHT = 0.65

# ★ 신규: 같은 장애물을 한 바퀴 넘게 도는 것을 감지하기 위한 회전각 누적 임계값/탈출 동작
FULL_LOOP_THRESH   = math.radians(400)   # 약 400도 누적 회전 -> "한 바퀴 돈 것 같다" 판단 (여유 40도)
LOOP_ESCAPE_BACK_V = -0.12               # 탈출 시 후진 속도
LOOP_ESCAPE_SEC    = 0.6                 # 후진 지속 시간(초)

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"   
mission_idx   = 0
detect_count  = 0
arrive_count  = 0
follow_side   = "L"   # 초기값일 뿐, WALL_SEARCH -> WALL_APPROACH 전환 시 decide_follow_side()로 항상 재결정됨
lidar_state   = "WALL_SEARCH"

park_state    = "TRACK"   
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
search_t      = None      
mission_start_t = time.time() 
hop_start_t     = None        
last_cmd      = (0.0, 0.0)

# ★ 신규: 벽 추종 중 벽을 놓쳤을 때(WALL_LOST) 제자리 회전으로 다시 찾는 동안
#   누적 회전각을 추적하기 위한 상태. 한 바퀴(FULL_LOOP_THRESH) 돌아도 못 찾으면
#   WALL_SEARCH로 폴백한다.
wall_lost_accum_rad = 0.0
last_loop_t          = time.time()

print(f"START | MISSION: {MISSION}")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = W // 2
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        # ★ 신규: 벽 놓침(WALL_LOST) 누적 회전각 계산용 프레임 간 시간 차
        now_t   = time.time()
        loop_dt = now_t - last_loop_t
        last_loop_t = now_t

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        # ★ 화면 왼쪽 상단에 현재 추적 중인 색상 표시 (예: "TARGET: RED")
        # 해당 색상 고유의 BGR 색상으로 글씨가 나타납니다.
        cv2.putText(frame, f"TARGET: {target.upper()}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)

        is_searching = (mode == "LIDAR") or (mode == "PARK" and park_state in ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW", "SEARCH"])
        
        if is_searching:
            if time.time() - mission_start_t > MISSION_TIMEOUT_SEC:
                mode = "PARK"
                park_state = "SAFE_HOP"
                hop_start_t = time.time()
                detect_count = 0
                continue
        elif park_state not in ["SAFE_HOP"]:
            mission_start_t = time.time()

        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big    = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        cx_obj, cy_obj = -1, -1
        if found:
            M_mom = cv2.moments(big)
            if M_mom["m00"] > 0:
                cx_obj = int(M_mom["m10"] / M_mom["m00"])
                cy_obj = int(M_mom["m01"] / M_mom["m00"])
            bx, by_top, bw, bh = cv2.boundingRect(big)
            last_seen_x   = cx_obj
            last_bottom_y = min(by_top + bh, 239)
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ★ 신규: 벽 추종 중 마지막으로 본 색지 방향의 부호만 사용 (단순 좌/우 판정)
        #   last_seen_x가 cx_mid보다 왼쪽이면 -1, 오른쪽이면 +1, 아직 한 번도 못 봤으면 0
        #   (last_seen_x 기본값은 cx_mid와 같은 160이므로 자연스럽게 0이 됨)
        def get_target_bias_sign():
            if last_seen_x < cx_mid: return -1
            if last_seen_x > cx_mid: return 1
            return 0

        # ══ LIDAR 모드 ═════════════════════════════════════════════════════
        if mode == "LIDAR":
            if found: detect_count += 1
            else: detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                continue

            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                    lidar_state = "WALL_APPROACH"
                else: send_cmd(0.0, WALL_SEARCH_W)

            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3: lidar_state = "WALL_FOLLOW"
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP: send_cmd(0.08, adir * 1.0)
                    elif fm < THRESH_TURN: send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7)
                    else: send_cmd(WALL_APPROACH_V, sign * 0.3)

            elif lidar_state == "WALL_FOLLOW":
                v, w, lost = wall_follow(scan, fm, adir, follow_side, get_target_bias_sign())
                if lost:
                    wall_lost_accum_rad += abs(w) * loop_dt
                    if wall_lost_accum_rad >= FULL_LOOP_THRESH:
                        wall_lost_accum_rad = 0.0
                        lidar_state = "WALL_SEARCH"
                        continue
                else:
                    wall_lost_accum_rad = 0.0
                send_cmd(v, w)

        # ══ PARK 모드 ═════════════════════════════════════════════════════
        elif mode == "PARK":

            if park_state == "SAFE_HOP":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        mission_start_t = time.time()
                        continue
                else: detect_count = 0

                elapsed_hop = time.time() - hop_start_t
                
                if elapsed_hop < 2.0:
                    send_cmd(0.0, 1.2)
                else:
                    if fm > 130.0:
                        send_cmd(0.0, 1.0)
                    elif fm > 50.0:
                        send_cmd(WALL_V, 0.0)
                    else:
                        follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                        park_state = "WALL_APPROACH"
                        mission_start_t = time.time()
                        continue
                
                cv2.imshow("f", frame); cv2.waitKey(1); continue


            if park_state == "FORWARD":
                elapsed = time.time() - park_t
                if elapsed >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()
                else: send_cmd(*last_cmd)

            elif park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                    continue

            elif park_state == "WALL_SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        continue
                else: detect_count = 0

                if fm < WALL_SCAN_DIST:
                    follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                    park_state = "WALL_APPROACH"
                    continue
                send_cmd(0.0, WALL_SEARCH_W)

            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else: detect_count = 0

                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3: park_state = "WALL_FOLLOW"; continue

                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP: v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN: v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else:
                    v = WALL_APPROACH_V
                    w = sign * 0.3
                    # ★ 신규: 벽 접근 중에도 마지막으로 본 색지 방향으로 회전각 편향
                    tb = get_target_bias_sign()
                    if tb != 0:
                        w = (1.0 - TARGET_BIAS_WEIGHT) * w + TARGET_BIAS_WEIGHT * (tb * 0.6)
                        w = float(np.clip(w, -0.9, 0.9))
                send_cmd(v, w)

            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else: detect_count = 0

                v, w, lost = wall_follow(scan, fm, adir, follow_side, get_target_bias_sign())
                if lost:
                    wall_lost_accum_rad += abs(w) * loop_dt
                    if wall_lost_accum_rad >= FULL_LOOP_THRESH:
                        wall_lost_accum_rad = 0.0
                        park_state = "WALL_SEARCH"
                        continue
                else:
                    wall_lost_accum_rad = 0.0
                send_cmd(v, w)

            elif park_state == "TRACK":
                park_state = "TRACK"

                # ★ 신규: 색지를 추적하기 전에, 색지 방향에 장애물이 있는지 먼저 확인
                #   (색지와 로봇 사이에 장애물이 있으면 그쪽으로 접근하지 않고 우회 상태로 전환)
                if found:
                    target_angle = cx_to_lidar_angle(cx_obj, cx_mid, W)
                    target_clear = obstacle_on_target(scan, target_angle)
                    if target_clear < TARGET_CLEAR_DIST:
                        # 색지 방향이 막혀 있음 -> TRACK 중단, 장애물 우회로 전환
                        arrive_count = 0
                        follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                        park_state = "WALL_APPROACH"
                        continue

                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0; park_state = "FORWARD"; park_t = time.time()
                    send_cmd(ARRIVE_FORWARD_V, 0.0); continue
                else:
                    err_x = cx_obj - cx_mid
                    err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                    reduced_v = APPROACH_V * (1.0 - err_ratio)

                    def cam_w(ex):
                        raw = -KP_ROT * ex
                        if abs(raw) < W_MIN and ex != 0: return -W_MIN if ex > 0 else W_MIN
                        return raw

                    if fm >= THRESH_SLOW: v, w = reduced_v, cam_w(err_x)
                    else:
                        w_cam = cam_w(err_x); w_lid = adir * 0.7
                        if fm < THRESH_STOP: v, w = 0.09, w_lid
                        elif fm < THRESH_TURN: v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                        else: v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                    last_cmd = (v, w); send_cmd(v, w)

            elif park_state == "SEARCH":
                if park_state != "SEARCH":
                    park_state = "SEARCH"; search_t = time.time(); arrive_count = 0
                elapsed_search = time.time() - search_t
                if elapsed_search > 5.0:
                    park_state = "WALL_SEARCH"
                else:
                    v = 0.0; w = (-1.0 if last_seen_x > cx_mid else 1.0)
                    send_cmd(v, w)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
