# 노,파 추적 강화
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
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

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

def side_min_follow(scan, side):
    # follow_side 쪽 "벽이 진짜 있는지" 판단용 최솟값 (노이즈에 강함)
    if side == "L":
        return side_min(scan, 85, 96)
    else:
        return side_min(scan, 265, 276)

# ── WALL-FOLLOW STATE (모서리/통로 노이즈 대응) ─────────────────────
WALL_LOST_THRESH  = 70.0   # side_min 기준, 이 거리 넘게 벽이 없다고 판단되면 LOST
WALL_LOST_CONFIRM = 5      # 연속 프레임 확인 후에만 LOST 전환 (노이즈 1~2프레임 무시)
_lost_streak = {"L": 0, "R": 0}

def wall_follow(scan, fm, adir, follow_side):
    sd          = side_dist(scan, follow_side)         # 제어용 (평균, 부드러움)
    sd_min      = side_min_follow(scan, follow_side)    # LOST 판단용 (최솟값, 노이즈에 강함)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1

    # ① 정면 위험 → 긴급 회피. 단, follow_side 쪽으로 편향시켜서
    #    회피 후에도 따라가던 벽에서 너무 멀어지지 않게 한다.
    if fm < THRESH_STOP:
        w = 0.6 * (adir * 1.1) + 0.4 * (sign * 1.1)
        return (0.08, float(np.clip(w, -1.4, 1.4)))
    if fm < THRESH_TURN:
        w = 0.6 * (adir * 0.85) + 0.4 * (sign * 0.85)
        return (WALL_TURN_V, float(np.clip(w, -1.2, 1.2)))

    # ② 측면 너무 가까움 (모서리에 끼는 상황) → 반대쪽으로
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    # ③ 따라가는 쪽 벽을 "진짜" 잃었는지 확인 (최솟값 + 연속 프레임)
    #    평균(sd)만 보면 모서리/통로에서 순간적으로 튀어 오발동하므로
    #    side_min이 임계값을 WALL_LOST_CONFIRM 프레임 연속으로 넘을 때만 LOST 처리.
    if sd_min > WALL_LOST_THRESH:
        _lost_streak[follow_side] += 1
    else:
        _lost_streak[follow_side] = 0

    if _lost_streak[follow_side] >= WALL_LOST_CONFIRM:
        # 벽을 진짜 잃음 → follow_side 쪽으로 더 적극적으로 틀면서 저속 탐색
        # (기존 WALL_LOST_W=0.5는 너무 약해 직진처럼 보였음 → 강화 + 속도 추가 감속)
        return (WALL_V * 0.5, sign * 0.9)

    # ④ 정상 wall-following
    err = sd - WALL_TARGET
    w   = sign * WALL_KP * err
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
W_MIN          = 0.25
APPROACH_V     = 0.17
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

# 도착 판정 영역
ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 40
ARRIVE_FORWARD_SEC = 0.7
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# wall-following
WALL_TARGET   = 20.0
WALL_SCAN_DIST = 150.0  # 회전 중 정면(fm) 기준 벽 탐색 감지 거리 (cm)
WALL_APPROACH_V = 0.20  # 벽으로 접근할 때 속도
WALL_KP       = 0.012
WALL_V        = 0.22
WALL_TURN_V   = 0.10
WALL_LOST_W   = 0.5      # (참고용, 실제 LOST 처리는 wall_follow 내부 로직으로 대체됨)
WALL_SEARCH_W = 1.1     # 벽 탐색 제자리 회전 각속도

# WALL_APPROACH → WALL_FOLLOW 전환 시 노이즈 방지용 연속 프레임 확인
WALL_APPROACH_CONFIRM = 3
_approach_streak = {"LIDAR": 0, "PARK": 0}

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
        big   = max(cnts, key=cv2.contourArea) if cnts else None
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

        # 디버그용: 현재 follow_side 기준 거리/오차 표시 (화면 우측 상단)
        _dbg_sd  = side_dist(scan, follow_side)
        _dbg_min = side_min_follow(scan, follow_side)
        cv2.putText(frame, f"sd:{_dbg_sd:.0f} min:{_dbg_min:.0f} fm:{fm:.0f} lost:{_lost_streak[follow_side]}",
                    (W - 230, 20), 0, 0.45, (200, 200, 200), 1)

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
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    lidar_state = "WALL_APPROACH"
                    _approach_streak["LIDAR"] = 0
                    print(f"[LIDAR] 회전 중 벽 감지 fm:{fm:.0f}cm, follow_side={follow_side} → 접근 시작")
                else:
                    send_cmd(0.0, WALL_SEARCH_W)
                    cv2.putText(frame, f"LIDAR-WALL-SEARCH fm:{fm:.0f}",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── B. 벽으로 접근 중 (WALL_APPROACH) ─────────────────
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)

                # 연속 프레임 확인 후 전환 (비스듬한 접근 중 일시적으로 가까워지는 노이즈 방지)
                if sd <= WALL_TARGET * 1.3:
                    _approach_streak["LIDAR"] += 1
                else:
                    _approach_streak["LIDAR"] = 0

                if _approach_streak["LIDAR"] >= WALL_APPROACH_CONFIRM:
                    lidar_state = "WALL_FOLLOW"
                    _lost_streak[follow_side] = 0
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
                cv2.putText(frame, f"LIDAR-WALL-FOLLOW {follow_side}:{sd_disp:.0f}cm v:{v:.2f} w:{w:.2f}",
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
                        _lost_streak["L"] = 0
                        _lost_streak["R"] = 0
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
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    park_state = "WALL_APPROACH"
                    _approach_streak["PARK"] = 0
                    print(f"회전 중 벽 감지 fm:{fm:.0f}cm, follow_side={follow_side} → 접근 시작")
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
                # (연속 프레임 확인으로 비스듬한 접근 중 노이즈 전환 방지)
                if sd <= WALL_TARGET * 1.3:
                    _approach_streak["PARK"] += 1
                else:
                    _approach_streak["PARK"] = 0

                if _approach_streak["PARK"] >= WALL_APPROACH_CONFIRM:
                    park_state = "WALL_FOLLOW"
                    _lost_streak[follow_side] = 0
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
                cv2.putText(frame, f"WALL-FOLLOW [{target}] {follow_side}:{sd_disp:.0f}cm v:{v:.2f} w:{w:.2f}",
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

                    # 디버그: fm 임계값에 따라 카메라 조향이 라이다 회피값에
                    # 얼마나 잠식당하는지 확인용. 색깔별로 fm이 낮게 유지되면
                    # (THRESH_SLOW 미만) w_cam이 깎이는 게 원인일 가능성이 큼.
                    if fm >= THRESH_SLOW:
                        dbg = f"fm:{fm:.0f}(SLOW-OK) errx:{err_x} w_cam:{w:.2f}"
                    else:
                        dbg = f"fm:{fm:.0f}(LIDAR-MIX) errx:{err_x} w_lid:{w_lid:.2f} w_cam:{w_cam:.2f} w_final:{w:.2f}"
                    cv2.putText(frame, dbg, (10, H - 10), 0, 0.45, (255, 255, 0), 1)

                cv2.putText(frame, f"TRACKING: {target}", (10, 25), 0, 0.6, draw, 1)

            # ── 5. 객체 놓침 / 제자리 탐색 (SEARCH) ───────────────
            else:
                park_state = "SEARCH"
                arrive_count = 0
                v = 0.0
                w = (-1.0 if last_seen_x > cx_mid else 1.0)
                send_cmd(v, w)
                cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
