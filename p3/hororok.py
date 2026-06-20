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

def side_dist(scan, side):
    if side == "L":
        idx = np.arange(85, 96) % 360
    else:
        idx = np.arange(265, 276) % 360
    return float(np.mean(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.030
W_MIN          = 0.20
APPROACH_V     = 0.13
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP        = int(240 * 0.85)
ARRIVE_X_MARGIN     = 30
ARRIVE_FORWARD_SEC  = 0.8
ARRIVE_FORWARD_V    = 0.13
ARRIVE_CONFIRM      = 8

WALL_TARGET     = 20.0
WALL_SCAN_DIST  = 150.0
WALL_APPROACH_V = 0.20
WALL_KP         = 0.018   # 평균 거리 오차 게인
WALL_KU         = 0.025   # 균일도(평행도) 오차 게인
WALL_V          = 0.22
WALL_TURN_V     = 0.10
WALL_LOST_W     = 0.5
WALL_SEARCH_W   = 1.1

MISSION_TIMEOUT_SEC = 10.0

# ── WALL FOLLOW (균일 거리 유지 방식) ────────────────────────────────
def wall_follow(scan, fm, adir, follow_side):
    """
    왼쪽 추종 : 271~281도 슬라이스가 모두 WALL_TARGET에 균일하게 수렴하도록 w 조정
    오른쪽 추종: 80~90도 슬라이스가 모두 WALL_TARGET에 균일하게 수렴하도록 w 조정

    오차 항:
      err_dist    : mean(wall_vals) - WALL_TARGET  → 평균 거리 오차
      err_uniform : wall_vals[0] - wall_vals[-1]   → 앞/뒤 각도 거리 비대칭(평행도) 오차
    """
    if follow_side == "L":
        idx  = np.arange(271, 282) % 360   # 271~281도 (11포인트)
        sign = 1                             # 왼쪽으로 조향 시 양수
    else:
        idx  = np.arange(80, 91) % 360      # 80~90도 (11포인트)
        sign = -1                            # 오른쪽으로 조향 시 음수

    wall_vals = scan[idx]                    # 해당 범위의 거리값 배열
    mean_dist = float(np.mean(wall_vals))

    # ── 장애물 유실 판단 ──────────────────────────────────────────────
    if mean_dist > WALL_TARGET * 2.5:
        if fm < THRESH_STOP:
            return (0.08, adir * 1.1)
        return (WALL_V * 0.65, sign * WALL_LOST_W)

    # ── 전방/측방 장애물 회피 우선 ───────────────────────────────────
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)

    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    # ── 오차 계산 ────────────────────────────────────────────────────
    # 오차1: 평균 거리 vs 목표 거리
    #   양수 → 벽에서 멀어짐 → sign 방향(벽 쪽)으로 조향
    err_dist = mean_dist - WALL_TARGET

    # 오차2: 균일도(평행도)
    #   wall_vals[0]  : idx 배열 첫 요소 = 로봇 전방 쪽에 가까운 측면 각도
    #   wall_vals[-1] : idx 배열 끝 요소 = 로봇 후방 쪽에 가까운 측면 각도
    #   양수 → 앞쪽이 벽에서 더 멈(앞머리가 벌어짐) → 앞머리를 벽 쪽으로 틀어야 함
    #   음수 → 앞쪽이 벽에 더 가까움(앞머리가 파고듦) → 앞머리를 벽에서 멀리 틀어야 함
    err_uniform = float(wall_vals[0] - wall_vals[-1])

    # 최종 조향: 두 오차의 합산
    w = sign * (WALL_KP * err_dist + WALL_KU * err_uniform)

    # ── 전방 장애물 접근 시 속도/조향 블렌딩 ────────────────────────
    if fm < THRESH_SLOW:
        blend = float(np.clip(
            (THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0
        ))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V

    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)

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

# ── STATE ─────────────────────────────────────────────────────────────
mode            = "LIDAR"
mission_idx     = 0
detect_count    = 0
arrive_count    = 0
follow_side     = "L"       # 왼쪽 벽 추종 고정
lidar_state     = "WALL_SEARCH"

park_state      = "TRACK"
last_seen_x     = 160
last_bottom_y   = 0
park_t          = None
search_t        = None
mission_start_t = time.time()
hop_start_t     = None
last_cmd        = (0.0, 0.0)

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
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        # ── 10초 타임아웃 감시 ────────────────────────────────────────
        is_searching = (mode == "LIDAR") or (mode == "PARK" and park_state in ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW", "SEARCH"])

        if is_searching:
            if time.time() - mission_start_t > MISSION_TIMEOUT_SEC:
                print(f"🚨 [{target}] {MISSION_TIMEOUT_SEC}초 경과! 다른 장애물로 건너뜁니다.")
                mode        = "PARK"
                park_state  = "SAFE_HOP"
                hop_start_t = time.time()
                detect_count = 0
                continue
        elif park_state not in ["SAFE_HOP"]:
            mission_start_t = time.time()

        # ── 색상 검출 ─────────────────────────────────────────────────
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

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ══ LIDAR 모드 ════════════════════════════════════════════════
        if mode == "LIDAR":
            if found: detect_count += 1
            else:     detect_count  = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode       = "PARK"
                park_state = "TRACK"
                continue

            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    follow_side = "L"
                    lidar_state = "WALL_APPROACH"
                else:
                    send_cmd(0.0, WALL_SEARCH_W)

            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    lidar_state = "WALL_FOLLOW"
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:        send_cmd(0.08, adir * 1.0)
                    elif fm < THRESH_TURN:      send_cmd(WALL_APPROACH_V * 0.6, adir * 0.7)
                    else:                       send_cmd(WALL_APPROACH_V, sign * 0.3)

            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)

        # ══ PARK 모드 ═════════════════════════════════════════════════
        elif mode == "PARK":

            # ── SAFE_HOP: 맵 이탈 방지 안전 도약 ────────────────────
            if park_state == "SAFE_HOP":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        print(f"[{target}] 도약 중 타겟 발견! 추적 시작")
                        detect_count    = 0
                        park_state      = "TRACK"
                        mission_start_t = time.time()
                        continue
                else:
                    detect_count = 0

                elapsed_hop = time.time() - hop_start_t

                if elapsed_hop < 2.0:
                    # 단계1: 기존 벽 외면 — 제자리 회전
                    send_cmd(0.0, 1.2)
                    cv2.putText(frame, "HOP: TURNING AWAY", (10, 45), 0, 0.5, (0, 0, 255), 2)
                else:
                    if fm > 130.0:
                        # 허공/맵 밖 → 회전 유지
                        send_cmd(0.0, 1.0)
                        cv2.putText(frame, "HOP: SCANNING MAP INWARD", (10, 45), 0, 0.5, (0, 150, 255), 2)
                    elif fm > 50.0:
                        # 새 장애물 향해 직진
                        send_cmd(WALL_V, 0.0)
                        cv2.putText(frame, f"HOP: MOVING TO NEW ({fm:.0f}cm)", (10, 45), 0, 0.5, (0, 255, 0), 2)
                    else:
                        # 새 장애물 도착
                        print("   → 새로운 장애물 도착! 왼쪽 벽 탐색 재시작")
                        park_state      = "WALL_APPROACH"
                        mission_start_t = time.time()
                        continue

                cv2.imshow("f", frame); cv2.waitKey(1); continue

            # ── FORWARD ──────────────────────────────────────────────
            if park_state == "FORWARD":
                elapsed = time.time() - park_t
                if elapsed >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t     = time.time()
                else:
                    send_cmd(*last_cmd)

            # ── PARKING ──────────────────────────────────────────────
            elif park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC:
                    mission_idx  += 1
                    arrive_count  = 0
                    detect_count  = 0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                    continue

            # ── WALL_SEARCH ──────────────────────────────────────────
            elif park_state == "WALL_SEARCH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state   = "TRACK"
                        continue
                else:
                    detect_count = 0

                if fm < WALL_SCAN_DIST:
                    park_state = "WALL_APPROACH"
                    continue
                send_cmd(0.0, WALL_SEARCH_W)

            # ── WALL_APPROACH ─────────────────────────────────────────
            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state   = "TRACK"
                        continue
                else:
                    detect_count = 0

                sd   = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    park_state = "WALL_FOLLOW"
                    continue

                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP:        v, w = 0.08, adir * 1.0
                elif fm < THRESH_TURN:      v, w = WALL_APPROACH_V * 0.6, adir * 0.7
                else:                       v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)

            # ── WALL_FOLLOW ───────────────────────────────────────────
            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state   = "TRACK"
                        continue
                else:
                    detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)

            # ── TRACK ─────────────────────────────────────────────────
            elif park_state == "TRACK":
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state   = "FORWARD"
                    park_t       = time.time()
                    send_cmd(ARRIVE_FORWARD_V, 0.0)
                    continue

                err_x     = cx_obj - cx_mid
                err_ratio = min(abs(err_x) / (cx_mid * 1.0), 1.0)
                reduced_v = APPROACH_V * (1.0 - err_ratio)

                def cam_w(ex):
                    raw = -KP_ROT * ex
                    if abs(raw) < W_MIN and ex != 0:
                        return -W_MIN if ex > 0 else W_MIN
                    return raw

                if fm >= THRESH_SLOW:
                    v, w = reduced_v, cam_w(err_x)
                else:
                    w_cam = cam_w(err_x)
                    w_lid = adir * 0.7
                    if fm < THRESH_STOP:        v, w = 0.09, w_lid
                    elif fm < THRESH_TURN:      v, w = 0.13, 0.7 * w_lid + 0.3 * w_cam
                    else:                       v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                last_cmd = (v, w)
                send_cmd(v, w)

            # ── SEARCH ───────────────────────────────────────────────
            elif park_state == "SEARCH":
                if search_t is None:
                    search_t     = time.time()
                    arrive_count = 0

                elapsed_search = time.time() - search_t
                if elapsed_search > 5.0:
                    park_state = "WALL_SEARCH"
                    search_t   = None
                else:
                    v = 0.0
                    w = -1.0 if last_seen_x > cx_mid else 1.0
                    send_cmd(v, w)

        # ── 디버그 오버레이 ───────────────────────────────────────────
        search_time_left = MISSION_TIMEOUT_SEC - (time.time() - mission_start_t)
        if is_searching and search_time_left > 0:
            cv2.putText(frame, f"Time: {search_time_left:.1f}s | Side: {follow_side}",
                        (10, 20), 0, 0.55, (255, 150, 0), 2)

        # 벽 추종 중 균일도 디버그 표시
        if (mode == "LIDAR" and lidar_state == "WALL_FOLLOW") or \
           (mode == "PARK"  and park_state  == "WALL_FOLLOW"):
            if follow_side == "L":
                idx = np.arange(271, 282) % 360
            else:
                idx = np.arange(80, 91) % 360
            wv        = scan[idx]
            mean_d    = float(np.mean(wv))
            uniformity = float(wv[0] - wv[-1])
            cv2.putText(frame,
                        f"WF mean={mean_d:.1f} unif={uniformity:.1f}",
                        (10, H - 10), 0, 0.45, (0, 255, 200), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
