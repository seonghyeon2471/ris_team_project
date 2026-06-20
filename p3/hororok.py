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
FRONT_RANGE  = 90   # 정면 감지 범위 (±90도 = 총 180도)
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
    # 평균(np.mean) 대신 하위 30퍼센타일을 사용해 한두 개의 튀는(먼 거리) 값에
    # 덜 민감하게 만들고, 각 구역에서 "비교적 가까운 쪽" 장애물 거리를 대표하도록 함.
    # scan[1:90]   → 왼쪽 구역 (0~90도)
    # scan[271:360]→ 오른쪽 구역 (271~360도)
    left_p  = np.percentile(scan[1:90],   30)
    right_p = np.percentile(scan[271:360], 30)
    return 1 if left_p >= right_p else -1

def side_dist(scan, side):
    # side: "L" → 85~95도(왼쪽), "R" → 265~275도(오른쪽)
    if side == "L":
        idx = np.arange(85, 96) % 360
    else:
        idx = np.arange(265, 276) % 360
    return float(np.mean(scan[idx]))

def left_dist(scan):
    return side_dist(scan, "L")

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ── WALL PARALLEL (평행 유지용 미분 상태) ────────────────────────────
# side_dist 1개 지점의 시간당 변화율(slope)로 벽에 대한 로봇의 각도를 추정.
# slope > 0 → 거리가 점점 멀어지는 중 → 로봇 앞쪽이 벽에서 벌어지는 방향(비스듬)
# slope < 0 → 거리가 점점 가까워지는 중 → 로봇 앞쪽이 벽 쪽으로 파고드는 방향
# slope ≈ 0 → 거리 변화 없음 → 벽과 평행하게 주행 중
WALL_KD        = 0.018   # 평행도(slope) 보정 게인 — 클수록 평행 정렬을 강하게 함
SLOPE_ALPHA    = 0.4     # slope 값 EMA 스무딩 계수 (라이다 노이즈 완화)
SLOPE_MAX      = 8.0     # cm/s, 이상치 클램프

_sd_prev       = {"L": None, "R": None}
_slope_ema     = {"L": 0.0, "R": 0.0}
_sd_t_prev     = {"L": None, "R": None}

def _update_wall_slope(sd, follow_side):
    """side_dist 변화율을 시간 정규화 + EMA 스무딩해서 반환 (cm/s)."""
    now = time.time()
    prev_sd = _sd_prev[follow_side]
    prev_t  = _sd_t_prev[follow_side]

    if prev_sd is not None and prev_t is not None:
        dt = now - prev_t
        if dt > 1e-3:
            raw_slope = (sd - prev_sd) / dt
            raw_slope = float(np.clip(raw_slope, -SLOPE_MAX, SLOPE_MAX))
            _slope_ema[follow_side] = (
                (1 - SLOPE_ALPHA) * _slope_ema[follow_side] + SLOPE_ALPHA * raw_slope
            )

    _sd_prev[follow_side]   = sd
    _sd_t_prev[follow_side] = now
    return _slope_ema[follow_side]

def reset_wall_slope(side=None):
    """WALL_SEARCH/WALL_APPROACH로 돌아가거나 follow_side가 바뀔 때 호출해서
    오래된 미분 상태(다른 위치/시점의 거리값)가 다음 추종에 섞이지 않게 함."""
    sides = [side] if side else ["L", "R"]
    for s in sides:
        _sd_prev[s]   = None
        _slope_ema[s] = 0.0
        _sd_t_prev[s] = None

def wall_follow(scan, fm, adir, follow_side):
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    diag_left   = side_min(scan, 35, 80)
    diag_right  = side_min(scan, 280, 325)
    sign = 1 if follow_side == "L" else -1

    # ① 정면 위험 → adir 긴급 회피
    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    # ② 측면 너무 가까움 → 반대쪽으로
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    # ③ 바깥 코너 감지 → 따라가던 벽이 갑자기 멀어지고 대각선 방향도 비어있음
    #    (sd > 2.5배 "완전 상실" 조건보다 먼저 검사해야 의미가 있음 — 아래 ④ 참고)
    if follow_side == "L" and diag_left > 50 and sd > WALL_TARGET * 1.5:
        return (WALL_V * 0.6, 0.8)
    if follow_side == "R" and diag_right > 50 and sd > WALL_TARGET * 1.5:
        return (WALL_V * 0.6, -0.8)

    # ④ 벽 완전 상실 → 반대쪽으로 크게 돌며 재탐색
    #    주의: sd > WALL_TARGET*2.5 는 sd > WALL_TARGET*2.0 (⑤번)을 항상 포함하는
    #    상위 집합이므로, 반드시 ⑤번보다 먼저 검사해야 함.
    #    순서가 바뀌면 이 분기는 절대 실행되지 않음.
    if sd > WALL_TARGET * 2.5:
        return (WALL_V * 0.5, sign * WALL_LOST_W)

    # ⑤ 따라가는 쪽 장애물 없음 → 그쪽으로 돌아서 찾기
    if sd > WALL_TARGET * 2.0:
        return (WALL_V * 0.7, sign * WALL_LOST_W)

    # ⑥ 정상 wall-following — 거리 유지(P) + 평행 유지(D, slope) 동시 제어
    #    err   : 목표 거리와의 차이 → 가깝거나 멀면 보정 (P항)
    #    slope : side_dist 변화율 → 0이 아니면 벽에 대해 비스듬한 상태 → 평행하게 보정 (D항)
    err   = sd - WALL_TARGET
    slope = _update_wall_slope(sd, follow_side)
    w     = sign * (WALL_KP * err + WALL_KD * slope)
    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V
    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]),
               "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),
               "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]),
               "hsv2": None,
               "bgr":  ([40,  0,   0], [255, 220, 220]), "draw": (255, 80, 0)},
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

# 도착 판정 영역
ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

# wall-following
WALL_TARGET    = 10.0
WALL_SCAN_DIST = 150.0  # 회전 중 정면(fm) 기준 벽 탐색 감지 거리 (cm)
WALL_APPROACH_V = 0.20  # 벽으로 접근할 때 속도
WALL_KP        = 0.012
WALL_V         = 0.22
WALL_TURN_V    = 0.10
WALL_LOST_W    = 0.5
WALL_SEARCH_W  = 1.1     # 벽 탐색 제자리 회전 각속도

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"   # LIDAR | PARK
mission_idx   = 0
detect_count  = 0
arrive_count  = 0

# 어느 쪽 벽을 따라갈지 ("L" | "R") — 벽 감지 시점에 결정
follow_side   = "L"

# LIDAR 모드에서도 PARK과 동일한 wall 탐색 상태머신을 사용
# lidar_state: WALL_SEARCH | WALL_APPROACH | WALL_FOLLOW
lidar_state   = "WALL_SEARCH"

park_state    = "TRACK"   # TRACK | WALL_SEARCH | WALL_APPROACH | WALL_FOLLOW | FORWARD | PARKING
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
search_t      = None      # ★ 탐색 타이머 변수 추가
last_cmd      = (0.0, 0.0)

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

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

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
            cv2.line(frame, (cx_obj, by_top), (cx_obj, by_top + bh), (0, 255, 255), 1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and
                    cy_obj >= ARRIVE_Y_TOP)

        # ══ LIDAR 모드 (첫 번째 색지 탐색, wall-following 포함) ══
        if mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                print(f"[{target}] 발견 → 추적 시작")
                continue

            # ── A. 벽 탐색 중 제자리 회전 (WALL_SEARCH) ──────────
            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    follow_side = "L"
                    lidar_state = "WALL_APPROACH"
                    print(f"[LIDAR] 회전 중 벽 감지 fm:{fm:.0f}cm, 무조건 왼쪽(L) 접근 시작")
                else:
                    send_cmd(0.0, WALL_SEARCH_W)
                    cv2.putText(frame, f"LIDAR-WALL-SEARCH fm:{fm:.0f}",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── B. 벽으로 접근 중 (WALL_APPROACH) ─────────────────
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)

                if sd <= WALL_TARGET * 1.3:
                    lidar_state = "WALL_FOLLOW"
                    reset_wall_slope(follow_side)
                    print(f"[LIDAR] 벽 도달 {follow_side}:{sd:.0f}cm → wall-following 시작")
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:
                        v, w = 0.08, adir * 1.0
                    elif fm < THRESH_TURN:
                        v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                    else:
                        v, w = WALL_APPROACH_V, sign * 0.3
                    send_cmd(v, w)
                    cv2.putText(frame, f"LIDAR-WALL-APPROACH {follow_side}:{sd:.0f}cm",
                                (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── C. wall-following으로 탐색 (WALL_FOLLOW) ─────────
            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                cv2.putText(frame, f"LIDAR-WALL-FOLLOW {follow_side}:{sd_disp:.0f}cm",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            cv2.putText(frame, "MODE: LIDAR", (10, 45), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드 ════════════════════════════════════════════
        elif mode == "PARK":

            # ── 1. 도착 후 전진 중 (FORWARD) ──────────────────────
            if park_state == "FORWARD":
                elapsed = time.time() - park_t
                if elapsed >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()
                    print(f"[{target}] 전진 완료 → 정차")
                else:
                    send_cmd(*last_cmd)
                cv2.putText(frame, f"FORWARD: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 2. 정차 중 (PARKING) ───────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"   # ← wall-following으로 전환
                        print(f"다음 미션 [{MISSION[mission_idx]}] wall-following 탐색 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 3-A. 벽 탐색 중 제자리 회전 (WALL_SEARCH) ────────
            elif park_state == "WALL_SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 탐색 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                # 회전하면서 정면(front_min) 기준으로 멀리 있는 벽 감지
                if fm < WALL_SCAN_DIST:
                    follow_side = "L"
                    park_state = "WALL_APPROACH"
                    print(f"회전 중 벽 감지 fm:{fm:.0f}cm, 무조건 왼쪽(L) 접근 시작")
                    continue

                # 아직 아무것도 없음 → 제자리 좌회전으로 탐색
                send_cmd(0.0, WALL_SEARCH_W)
                cv2.putText(frame, f"WALL-SEARCH [{target}] 회전 중 fm:{fm:.0f}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 3-B. 벽으로 접근 중 (WALL_APPROACH) ───────────────
            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] 접근 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                sd = side_dist(scan, follow_side)

                # follow_side 거리가 WALL_TARGET에 도달 → wall-following 시작
                if sd <= WALL_TARGET * 1.3:
                    park_state = "WALL_FOLLOW"
                    reset_wall_slope(follow_side)
                    print(f"벽 도달 {follow_side}:{sd:.0f}cm → wall-following 시작")
                    continue

                # 정면 막히면 adir 회피하며 접근, 아니면 follow_side 쪽으로 틀며 전진
                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP:
                    v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN:
                    v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else:
                    v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)
                cv2.putText(frame, f"WALL-APPROACH [{target}] {follow_side}:{sd:.0f}cm",
                            (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── 3-C. wall-following으로 탐색 (WALL_FOLLOW) ─────────
            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        print(f"[{target}] wall-following 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)
                sd_disp = side_dist(scan, follow_side)
                cv2.putText(frame, f"WALL-FOLLOW [{target}] {follow_side}:{sd_disp:.0f}cm",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 4. 객체 추적 중 (TRACK) ────────────────────────────
            elif found:
                park_state = "TRACK"
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] centroid {ARRIVE_CONFIRM}프레임 확정 → {ARRIVE_FORWARD_SEC}초 전진")
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue

                else:
                    err_x = cx_obj - cx_mid
                    err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                    reduced_v = APPROACH_V * (1.0 - err_ratio)

                    def cam_w(ex):
                        raw = -KP_ROT * ex
                        if abs(raw) < W_MIN and ex != 0:
                            return -W_MIN if ex > 0 else W_MIN
                        return raw

                    if fm >= THRESH_SLOW:
                        v = reduced_v
                        w = cam_w(err_x)
                    else:
                        w_cam = cam_w(err_x)
                        w_lid = adir * 0.7
                        if fm < THRESH_STOP:
                            v, w = 0.09, w_lid
                        elif fm < THRESH_TURN:
                            v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                        else:
                            v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                    last_cmd = (v, w)
                    send_cmd(v, w)

                cv2.putText(frame, f"TRACKING: {target}", (10, 25), 0, 0.6, draw, 1)

            # ── 5. 객체 놓침 / 제자리 탐색 (SEARCH) ───────────────
            else:
                # 방금 막 객체를 놓쳐서 SEARCH 모드로 진입한 경우 시간 기록
                if park_state != "SEARCH":
                    park_state = "SEARCH"
                    search_t = time.time()
                    arrive_count = 0

                elapsed_search = time.time() - search_t

                # 제자리 회전 시간이 5초를 초과했다면? -> 벽 타기로 복귀!
                if elapsed_search > 5.0:
                    print(f"[{target}] 5초 초과! 완전 놓침 → 벽 타며(WALL_SEARCH) 다시 탐색")
                    park_state = "WALL_SEARCH"
                    
                # 5초가 안 지났다면? -> 마지막 본 방향으로 제자리 회전하며 두리번거리기
                else:
                    v = 0.0
                    w = (-1.0 if last_seen_x > cx_mid else 1.0)
                    send_cmd(v, w)
                    # 화면에 몇 초째 찾고 있는지(elapsed_search) 표시
                    cv2.putText(frame, f"SEARCHING: {target} ({elapsed_search:.1f}s)", 
                                (10, 25), 0, 0.6, (0, 255, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
