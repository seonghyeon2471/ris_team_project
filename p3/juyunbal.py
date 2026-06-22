#준수
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
        idx = np.arange(65, 96) % 360
    else:
        idx = np.arange(265, 296) % 360
    return float(np.min(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ── 코너 감지용 보조 함수 ──────────────────────────────────────────────
#
# 로봇 기하 요약:
#   - 라이다: 몸체 맨앞에서 3cm 뒤 (= 로봇 기준 0° 방향 기준점)
#   - 바퀴축: 몸체 맨뒤에서 5cm 앞 = 맨앞에서 15cm 뒤 = 라이다에서 12cm 뒤
#   - 바퀴 외부 반폭: 20cm / 2 = 10cm (몸체보다 2cm 더 튀어남)
#
# 각도 기준: 0°=정면, 90°=왼쪽, 180°=후방, 270°=오른쪽

def front_diag_dist(scan, follow_side):
    """
    추종 벽 쪽 전방 대각선 거리 (약 45~50°).
    오목 코너 접근 시 이 값이 side_dist보다 급격히 작아짐.
    볼록 코너 회전 중 바퀴 외부가 벽 모서리에 걸리는 것도 이 구간에서 선행 감지.
    """
    if follow_side == "L":
        idx = np.arange(40, 66) % 360   # 전방 좌측 대각
    else:
        idx = np.arange(294, 320) % 360  # 전방 우측 대각
    return float(np.min(scan[idx]))

def rear_side_dist_follow(scan, follow_side):
    """
    추종 벽 쪽 후방 측면 거리 (약 120~140°).
    라이다에서 12cm 뒤에 있는 바퀴 근방의 벽/장애물 거리.
    볼록 코너를 돌 때 뒤쪽 바퀴가 모서리에 걸리기 직전 이 값이 급락함.
    """
    if follow_side == "L":
        idx = np.arange(110, 150) % 360  # 후방 좌측
    else:
        idx = np.arange(210, 250) % 360  # 후방 우측
    return float(np.min(scan[idx]))

def opp_front_diag_dist(scan, follow_side):
    """
    추종 벽 반대편 전방 대각선 거리.
    반대편에 돌출 장애물이 있을 때 이 값이 작아짐.
    """
    if follow_side == "L":
        idx = np.arange(294, 320) % 360  # 전방 우측 대각
    else:
        idx = np.arange(40, 66) % 360    # 전방 좌측 대각
    return float(np.min(scan[idx]))

# ── 코너 감지 파라미터 ─────────────────────────────────────────────────
#
# CORNER_DIAG_THRESH:
#   전방 대각선 거리가 이 값 미만이면 오목 코너(벽 안쪽 꺾임) 위험.
#   바퀴 외부 반폭(10cm) + 안전여유(14cm) = 24cm.  경험상 28cm 권장.
#
# CORNER_REAR_THRESH:
#   후방 측면 거리가 이 값 미만이면 바퀴가 모서리에 걸릴 위험.
#   바퀴 외부 반폭(10cm) + 안전여유(6cm) = 16cm.
#
# CONVEX_DETECT_RATIO:
#   side_dist / front_diag_dist 비율이 이 값 초과 →
#   전방 대각은 멀고 측면은 가까움 = 볼록 코너(벽 끝) 진입.
#   이때 급격한 WALL_LOST_W 회전 대신 완만하게 처리.

CORNER_DIAG_THRESH   = 28.0   # cm - 전방 대각 오목 코너 임계
CORNER_REAR_THRESH   = 16.0   # cm - 후방 바퀴 코너 임계
CONVEX_DETECT_RATIO  = 1.8    # side/diag 비율 (볼록 코너 감지)
CORNER_SLOW_V        = 0.10   # 코너 감지 시 감속 속도
CORNER_REAR_W        = 0.75   # 후방 바퀴 걸림 회피 각속도

def wall_follow(scan, fm, adir, follow_side):
    """
    벽 추종 + 코너 모서리 바퀴 걸림 방지.

    우선순위 (위에서 아래로):
      1. 정면 긴급 정지 / 회피
      2. 후방 바퀴 측면 걸림 방지  ← NEW
      3. 오목 코너 감지 (전방 대각↓) ← NEW
      4. 볼록 코너 진입 감지          ← 개선됨
      5. 양측 긴급 측면 회피
      6. 벽 소실 (볼록 코너 완전 진입)← 개선됨 (완만한 회전)
      7. 정상 PID 추종
    """
    sd         = side_dist(scan, follow_side)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1

    # ── 1. 정면 긴급 회피 ─────────────────────────────────────────────
    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)

    # ── 2. 후방 바퀴 측면 걸림 방지 ─────────────────────────────────
    # 볼록 코너를 돌 때, 라이다보다 12cm 뒤에 있는 바퀴가 모서리에 닿기 직전
    # 후방 측면 거리가 CORNER_REAR_THRESH 미만으로 급락함.
    # → 추종 벽 반대 방향으로 즉시 조타해서 바퀴를 모서리에서 이탈시킴.
    rear_sd = rear_side_dist_follow(scan, follow_side)
    if rear_sd < CORNER_REAR_THRESH:
        return (CORNER_SLOW_V, -sign * CORNER_REAR_W)

    # ── 3. 전방 대각: 오목 코너 (벽이 안쪽으로 꺾임) ────────────────
    # 추종 벽 쪽 45° 방향이 갑자기 막힘 → 오목 코너 접근.
    # 반대 방향으로 조타해 바퀴가 코너 안쪽에 끼이지 않게 함.
    fdiag = front_diag_dist(scan, follow_side)
    if fdiag < CORNER_DIAG_THRESH:
        # 얼마나 가까운지에 따라 조타량 비례 조절
        urgency = float(np.clip(
            (CORNER_DIAG_THRESH - fdiag) / CORNER_DIAG_THRESH, 0.0, 1.0
        ))
        w_escape = -sign * (0.4 + 0.5 * urgency)
        return (CORNER_SLOW_V, float(np.clip(w_escape, -0.9, 0.9)))

    # ── 4. 볼록 코너 진입 감지 (벽이 바깥쪽으로 끝남) ───────────────
    # side_dist는 아직 가깝지만, 전방 대각이 멀어지는 시점 =
    # 바퀴가 통과하기 직전 벽 모서리가 사라지는 순간.
    # 이때 급격히 sign 방향으로 꺾으면 뒷바퀴가 모서리에 걸림.
    # → 먼저 반대 방향으로 살짝 밀어 바퀴 간격 확보, 그 뒤 완만하게 전환.
    if sd < WALL_TARGET * 1.5 and fdiag > sd * CONVEX_DETECT_RATIO:
        # 전방 대각이 측면 거리의 CONVEX_DETECT_RATIO배를 넘음
        # = 코너가 이미 전방에 있고 바퀴가 진입 중
        w_pre = -sign * 0.35   # 잠깐 벽에서 멀어져 바퀴 이탈 공간 확보
        return (CORNER_SLOW_V, w_pre)

    # ── 5. 양측 긴급 측면 회피 ───────────────────────────────────────
    if left_close < THRESH_STOP:
        return (WALL_V * 0.7, -0.7)
    if right_close < THRESH_STOP:
        return (WALL_V * 0.7,  0.7)

    # ── 6. 벽 소실 (볼록 코너 완전 통과) ────────────────────────────
    # 기존: 즉시 WALL_LOST_W(=1.3) 급회전 → 뒷바퀴가 방금 지나온 모서리에 걸림.
    # 개선: 후방 바퀴가 안전한지 확인 후, 완만하게 단계적으로 회전.
    if sd > WALL_TARGET * 2.0:
        if rear_sd < CORNER_REAR_THRESH * 1.5:
            # 후방이 아직 코너 근처 → 더 완만하게 & 약간 반대로
            return (CORNER_SLOW_V, -sign * 0.3)
        else:
            # 후방 안전 → 벽 복귀 회전 (기존보다 완만하게)
            w_lost = sign * min(WALL_LOST_W, 0.9)
            return (0.05, w_lost)

    # ── 7. 정상 PID 벽 추종 ─────────────────────────────────────────
    err = sd - WALL_TARGET
    w   = sign * WALL_KP * err
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

WALL_TARGET    = 20.0
WALL_SCAN_DIST = 150.0
WALL_APPROACH_V = 0.20
WALL_KP        = 0.012
WALL_V         = 0.22
WALL_TURN_V    = 0.10
WALL_LOST_W    = 1.3
WALL_SEARCH_W  = 1.1
MISSION_TIMEOUT_SEC = 10.0

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0
arrive_count  = 0
follow_side   = "L"
lidar_state   = "WALL_SEARCH"

park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
search_t      = None
mission_start_t = time.time()
hop_start_t     = None
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
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

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
                    follow_side = "L"
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
                v, w = wall_follow(scan, fm, adir, follow_side)
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
                else: v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)

            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0; park_state = "TRACK"; continue
                else: detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd(v, w)

            elif park_state == "TRACK":
                park_state = "TRACK"
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
