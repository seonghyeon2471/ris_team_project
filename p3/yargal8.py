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
FRONT_RANGE  = 90   # 정면 감지 범위 (±90도 = 총 180도, 충돌 회피용 광각)
THRESH_SLOW  = 55.0
THRESH_TURN  = 30.0
THRESH_STOP  = 18.0

# 직진 중 "정면에 장애물이 있다/없다"를 판단할 때는 옆벽(추종 중인 벽)을
# 끌어들이지 않도록 광각(FRONT_RANGE)이 아니라 좁은 정면각을 따로 사용한다.
FRONT_NARROW_RANGE  = 30   # 정면 협각 (±30도)
OBSTACLE_NARROW_DIST = THRESH_SLOW  # 협각 기준 장애물 판정 거리 (cm)

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
    """광각(±90도) 정면 최소 거리 — 충돌 회피/감속 판단용 (그대로 유지)."""
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def front_min_narrow(scan, rng=FRONT_NARROW_RANGE):
    """협각(±30도) 정면 최소 거리 — '진짜 정면 장애물' 유무 판단용.
    벽 추종 중 옆벽(90도 부근)이 끼어들지 않도록 광각 front_min과 분리."""
    idx = np.arange(-rng, rng + 1) % 360
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

def wall_follow(scan, fm, follow_side):
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1

    # ① 정면 위험 → follow_side 반대쪽(벽에서 멀어지는 쪽)으로 회전하며 코너를 돌아나감.
    #    전역 adir 대신 항상 follow_side 기준으로 돌아야 코너에서도 "그 벽"을 계속 의식하며 감아 돈다.
    if fm < THRESH_STOP:
        return (0.08, -sign * 1.1)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, -sign * 0.85)

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
        w = (1 - blend) * w + blend * (-sign * 0.5)
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
APPROACH_V     = 0.17
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

# 도착 판정 영역
ARRIVE_Y_TOP    = int(240 * 0.85)
ARRIVE_X_MARGIN = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.15
ARRIVE_CONFIRM     = 8

# 직진 탐색 (GO_STRAIGHT)
GO_V = 0.20             # 직진 속도

# wall-following (장애물 외벽 추종 — yellow/blue 미션에서만 사용)
WALL_TARGET     = 20.0
WALL_KP         = 0.012
WALL_V          = 0.22
WALL_TURN_V     = 0.10
WALL_LOST_W     = 0.7

# ── STATE ─────────────────────────────────────────────────────────────
# state 종류:
# state 종류:
#   GO_STRAIGHT  : 직진하며 목표 색지 탐색 (라이다 단순 회피만 적용)
#   WALL_FOLLOW  : 정면 장애물 감지 → 그 장애물 외벽을 따라가며 탐색 (yellow/blue 전용)
#                  (멀면 다가가고, 가까우면 추종, 너무 가까우면 코너를 돌아나가는 동작까지 한 함수에서 처리)
#   TRACK        : 카메라로 색지를 추적
#   SEARCH       : 추적 중 색지를 놓쳐서 제자리 탐색
#   FORWARD      : 도착 판정 후 일정 시간 직진
#   PARKING      : 정차

state         = "GO_STRAIGHT"
mission_idx   = 0
detect_count  = 0
arrive_count  = 0

# 어느 쪽 벽을 따라갈지 ("L" | "R") — 장애물 감지 시점에 결정
follow_side   = "L"

park_t        = None
last_seen_x   = 160
last_bottom_y = 0
last_cmd      = (0.0, 0.0)

print(f"START | MISSION: {MISSION}")

# ── 직진 + 라이다 단순 회피 (벽 추종 없음, red 미션 및 평시 주행용) ──────
def go_straight_cmd(fm, adir):
    if fm < THRESH_STOP:
        return (0.08, adir * 1.1)
    elif fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 0.85)
    elif fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        v = GO_V * (1.0 - 0.5 * blend)
        w = adir * 0.5 * blend
        return (v, w)
    else:
        return (GO_V, 0.0)

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
        # "정면에 장애물이 있다"의 판정은 협각 기준 (옆벽 추종 중에도 오작동하지 않도록)
        narrow_blocked = front_min_narrow(scan) < OBSTACLE_NARROW_DIST

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

        # ══ GO_STRAIGHT: 직진하며 목표 색지 탐색 ════════════════════
        if state == "GO_STRAIGHT":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if mission_idx == 0:
                # ① red: 라이다는 단순 회피만 사용 (외벽 추종 없음)
                if detect_count >= DETECT_CONFIRM:
                    detect_count = 0
                    state = "TRACK"
                    print(f"[{target}] 발견 → 추적 시작")
                    continue

                v, w = go_straight_cmd(fm, adir)
                send_cmd(v, w)
                cv2.putText(frame, f"GO-STRAIGHT [{target}] fm:{fm:.0f}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)
            else:
                # ② yellow/blue: 색지 발견 + 정면 클리어 → 바로 추적
                if detect_count >= DETECT_CONFIRM and not narrow_blocked:
                    detect_count = 0
                    state = "TRACK"
                    print(f"[{target}] 발견 (정면 클리어) → 추적 시작")
                    continue

                # 정면에 장애물 인식 → 그 장애물 외벽 추종 시작 (접근까지 wall_follow가 처리)
                if narrow_blocked:
                    l = side_dist(scan, "L")
                    r = side_dist(scan, "R")
                    follow_side = "L" if l <= r else "R"
                    state = "WALL_FOLLOW"
                    print(f"[{target}] 탐색 중 정면 장애물 감지 → {follow_side}벽 추종 시작")
                    continue

                v, w = go_straight_cmd(fm, adir)
                send_cmd(v, w)
                cv2.putText(frame, f"GO-STRAIGHT [{target}] fm:{fm:.0f}",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

        # ══ WALL_FOLLOW: 장애물 외벽을 따라가며 탐색 (yellow/blue 전용) ═
        elif state == "WALL_FOLLOW":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM and not narrow_blocked:
                detect_count = 0
                state = "TRACK"
                print(f"[{target}] wall-following 중 발견 (정면 클리어) → 추적 시작")
                continue
            # 색지가 보여도 정면에 장애물이 남아있으면 카메라 인식을 무시하고
            # 계속 외벽을 따라간다 (위 조건의 not narrow_blocked 에서 자동 처리됨).

            v, w = wall_follow(scan, fm, follow_side)
            send_cmd(v, w)
            sd_disp = side_dist(scan, follow_side)
            cv2.putText(frame, f"WALL-FOLLOW [{target}] {follow_side}:{sd_disp:.0f}cm",
                        (10, 25), 0, 0.5, (0, 255, 0), 1)

        # ══ TRACK: 카메라로 색지 추적 ═══════════════════════════════
        elif state == "TRACK":
            if found:
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    state = "FORWARD"
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

            else:
                state = "SEARCH"
                arrive_count = 0
                v = 0.0
                w = (-1.0 if last_seen_x > cx_mid else 1.0)
                send_cmd(v, w)
                cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        # ══ SEARCH: 추적 중 놓친 색지를 제자리에서 다시 탐색 ════════
        elif state == "SEARCH":
            if found:
                state = "TRACK"
                continue
            v = 0.0
            w = (-1.0 if last_seen_x > cx_mid else 1.0)
            send_cmd(v, w)
            cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        # ══ FORWARD: 도착 판정 후 일정 시간 전진 ════════════════════
        elif state == "FORWARD":
            elapsed = time.time() - park_t
            if elapsed >= ARRIVE_FORWARD_SEC:
                stop_robot()
                state = "PARKING"
                park_t = time.time()
                print(f"[{target}] 전진 완료 → 정차")
            else:
                send_cmd(*last_cmd)
            cv2.putText(frame, f"FORWARD: {target}", (10, 25), 0, 0.6, draw, 2)

        # ══ PARKING: 정차, 다음 미션으로 전환 ════════════════════════
        elif state == "PARKING":
            stop_robot()
            elapsed = time.time() - park_t
            if elapsed >= PARK_SEC:
                mission_idx += 1
                arrive_count = 0
                detect_count = 0
                if mission_idx < len(MISSION):
                    state = "GO_STRAIGHT"
                    print(f"다음 미션 [{MISSION[mission_idx]}] 직진 탐색 시작")
                continue
            cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

        cv2.putText(frame, f"MISSION:{target} STATE:{state}", (10, 45),
                    0, 0.5, (255, 255, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
