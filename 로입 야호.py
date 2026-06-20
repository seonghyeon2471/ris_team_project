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
    # [FIX #1] 기존에는 read(5)가 타임아웃으로 5바이트를 못 채우면
    # 그 바이트들을 통째로 버려서, 한 번 어긋나면 영구 디싱크가 날 수 있었다.
    # → 누적 버퍼를 두고, 검증 실패 시 1바이트만 밀어서 재동기화한다.
    buf = bytearray()
    while True:
        chunk = lidar_ser.read(64)
        if chunk:
            buf.extend(chunk)
        else:
            continue

        while len(buf) >= 5:
            sf    = buf[0] & 0x01
            valid = (((buf[0] & 0x02) >> 1) == (1 - sf)
                     and (buf[1] & 0x01) == 1
                     and (buf[0] >> 2) >= 3)
            if not valid:
                del buf[0]          # 1바이트만 버리고 슬라이딩 재동기화
                continue

            raw     = buf[:5]
            angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
            dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
            if 3 < dist_cm < 150: _ema(angle, dist_cm)
            if sf == 1:
                _median()
                with scan_lock: _scan_pub[:] = _scan
            del buf[:5]

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

def wall_follow(scan, fm, adir, follow_side):
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
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

    # ③ 따라가는 쪽 장애물 없음 → 그쪽으로 돌아서 찾기
    if sd > WALL_TARGET * 2.0:
        return (WALL_V * 0.7, sign * WALL_LOST_W)

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

# ── WALL-FOLLOW 갇힘 감지 / 탈출 ─────────────────────────────────────────
# 고립된 장애물(기둥 등) 하나를 계속 따라 돌면 같은 자리를 영원히 맴돌 수 있다.
# wall_follow가 보낸 w(각속도)를 시간으로 적분해 누적 회전각을 추적하고,
# 한 바퀴(360°) 이상 돌면 "이 장애물에 갇혔다"고 보고 반대쪽으로 빠져나온다.
WALL_FOLLOW_STUCK_TURNS = 1.0    # 이 바퀴 수 이상 돌면 갇힌 것으로 판단
ESCAPE_SEC = 1.5                 # 탈출 기동(반대쪽 회전+전진) 지속 시간
ESCAPE_V   = 0.18                # 탈출 중 전진 속도
ESCAPE_W   = 0.8                 # 탈출 중 회전 각속도

wall_follow_heading_accum = 0.0
wall_follow_last_t        = None

def wall_follow_track_heading(w, now):
    """WALL_FOLLOW 중 누적 회전각을 추적. 한 바퀴 이상 돌면 True(갇힘) 반환."""
    global wall_follow_heading_accum, wall_follow_last_t
    if wall_follow_last_t is None:
        wall_follow_last_t = now
        return False
    dt = now - wall_follow_last_t
    wall_follow_last_t = now
    wall_follow_heading_accum += w * dt
    return abs(wall_follow_heading_accum) >= WALL_FOLLOW_STUCK_TURNS * 2 * math.pi

def wall_follow_heading_reset():
    global wall_follow_heading_accum, wall_follow_last_t
    wall_follow_heading_accum = 0.0
    wall_follow_last_t = None

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
APPROACH_V     = 0.17
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

# 도착 판정 영역
ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# wall-following
WALL_TARGET   = 20.0
WALL_SCAN_DIST = 150.0  # 회전 중 정면(fm) 기준 벽 탐색 감지 거리 (cm)
WALL_APPROACH_V = 0.20  # 벽으로 접근할 때 속도
WALL_KP       = 0.012
WALL_V        = 0.22
WALL_TURN_V   = 0.10
WALL_LOST_W   = 0.5
WALL_SEARCH_W = 1.1     # 벽 탐색 제자리 회전 각속도
WALL_APPROACH_KP_ANG = 0.012  # 가장 가까운 장애물 방향으로 회전할 때 P게인 (rad/s per deg)

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

park_state    = "TRACK"   # TRACK | WALL_SEARCH | WALL_APPROACH | WALL_FOLLOW | WALL_ESCAPE | FORWARD | PARKING
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
last_cmd      = (0.0, 0.0)
escape_t      = None

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
                # 정면 ±90도가 아니라 360도 전체 스캔에서 가장 가까운 점을 찾는다.
                near_dist  = float(np.min(scan))
                near_angle = int(np.argmin(scan))
                if near_dist < WALL_SCAN_DIST:
                    signed = near_angle if near_angle <= 180 else near_angle - 360
                    follow_side = "L" if signed >= 0 else "R"
                    lidar_state = "WALL_APPROACH"
                    print(f"[LIDAR] 가장 가까운 장애물 angle:{near_angle}, dist:{near_dist:.0f}cm → follow_side={follow_side} 접근 시작")
                else:
                    send_cmd(0.0, WALL_SEARCH_W)
                    cv2.putText(frame, f"LIDAR-WALL-SEARCH near:{near_dist:.0f}cm@{near_angle}",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── B. 벽으로 접근 중 (WALL_APPROACH) ─────────────────
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)

                if sd <= WALL_TARGET * 1.3:
                    lidar_state = "WALL_FOLLOW"
                    wall_follow_heading_reset()
                    print(f"[LIDAR] 벽 도달 {follow_side}:{sd:.0f}cm → wall-following 시작")
                else:
                    if fm < THRESH_STOP:
                        v, w = 0.08, adir * 1.0
                    elif fm < THRESH_TURN:
                        v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                    else:
                        near_angle = int(np.argmin(scan))
                        signed = near_angle if near_angle <= 180 else near_angle - 360
                        turn_ratio = min(abs(signed) / 90.0, 1.0)
                        v = WALL_APPROACH_V * (1.0 - 0.6 * turn_ratio)
                        w = float(np.clip(WALL_APPROACH_KP_ANG * signed, -0.9, 0.9))
                    send_cmd(v, w)
                    cv2.putText(frame, f"LIDAR-WALL-APPROACH {follow_side}:{sd:.0f}cm",
                                (10, 25), 0, 0.5, (0, 200, 0), 1)

            # ── C. wall-following으로 탐색 (WALL_FOLLOW) ─────────
            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                if wall_follow_track_heading(w, time.time()):
                    lidar_state = "WALL_ESCAPE"
                    escape_t = time.time()
                    wall_follow_heading_reset()
                    print("[LIDAR] 한 바퀴 이상 회전 → 같은 장애물에 갇힘 판단, 탈출 기동")
                else:
                    send_cmd(v, w)
                    sd_disp = side_dist(scan, follow_side)
                    cv2.putText(frame, f"LIDAR-WALL-FOLLOW {follow_side}:{sd_disp:.0f}cm",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── D. 갇힘 탈출 기동 (WALL_ESCAPE) ────────────────────
            elif lidar_state == "WALL_ESCAPE":
                elapsed = time.time() - escape_t
                if elapsed >= ESCAPE_SEC or fm < THRESH_STOP:
                    lidar_state = "WALL_SEARCH"
                    print("[LIDAR] 탈출 기동 종료 → 재탐색")
                else:
                    escape_w = -ESCAPE_W if follow_side == "L" else ESCAPE_W
                    send_cmd(ESCAPE_V, escape_w)
                    cv2.putText(frame, "LIDAR-WALL-ESCAPE", (10, 25), 0, 0.5, (0, 0, 255), 1)

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

                # 회전하면서 360도 전체 스캔에서 가장 가까운 장애물을 찾는다.
                near_dist  = float(np.min(scan))
                near_angle = int(np.argmin(scan))
                if near_dist < WALL_SCAN_DIST:
                    signed = near_angle if near_angle <= 180 else near_angle - 360
                    follow_side = "L" if signed >= 0 else "R"
                    park_state = "WALL_APPROACH"
                    print(f"가장 가까운 장애물 angle:{near_angle}, dist:{near_dist:.0f}cm → follow_side={follow_side} 접근 시작")
                    continue

                # 아직 아무것도 없음 → 제자리 좌회전으로 탐색
                send_cmd(0.0, WALL_SEARCH_W)
                cv2.putText(frame, f"WALL-SEARCH [{target}] 회전 중 near:{near_dist:.0f}cm@{near_angle}",
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
                    wall_follow_heading_reset()
                    print(f"벽 도달 {follow_side}:{sd:.0f}cm → wall-following 시작")
                    continue

                # 정면 막히면 adir 회피, 아니면 가장 가까운 점의 방향으로 비례 회전하며 전진
                if fm < THRESH_STOP:
                    v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN:
                    v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else:
                    near_angle = int(np.argmin(scan))
                    signed = near_angle if near_angle <= 180 else near_angle - 360
                    turn_ratio = min(abs(signed) / 90.0, 1.0)
                    v = WALL_APPROACH_V * (1.0 - 0.6 * turn_ratio)
                    w = float(np.clip(WALL_APPROACH_KP_ANG * signed, -0.9, 0.9))
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
                        wall_follow_heading_reset()
                        print(f"[{target}] wall-following 중 발견 → 추적 시작")
                        continue
                else:
                    detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side)
                if wall_follow_track_heading(w, time.time()):
                    park_state = "WALL_ESCAPE"
                    escape_t = time.time()
                    wall_follow_heading_reset()
                    print(f"[{target}] 한 바퀴 이상 회전 → 같은 장애물에 갇힘 판단, 탈출 기동")
                else:
                    send_cmd(v, w)
                    sd_disp = side_dist(scan, follow_side)
                    cv2.putText(frame, f"WALL-FOLLOW [{target}] {follow_side}:{sd_disp:.0f}cm",
                                (10, 25), 0, 0.5, (0, 255, 0), 1)

            # ── 3-D. 갇힘 탈출 기동 (WALL_ESCAPE) ──────────────────
            elif park_state == "WALL_ESCAPE":
                elapsed = time.time() - escape_t
                if elapsed >= ESCAPE_SEC or fm < THRESH_STOP:
                    park_state = "WALL_SEARCH"
                    print(f"[{target}] 탈출 기동 종료 → 재탐색")
                else:
                    escape_w = -ESCAPE_W if follow_side == "L" else ESCAPE_W
                    send_cmd(ESCAPE_V, escape_w)
                    cv2.putText(frame, f"WALL-ESCAPE [{target}]", (10, 25), 0, 0.5, (0, 0, 255), 1)

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
