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
FRONT_RANGE  = 45
THRESH_SLOW  = 55.0
THRESH_TURN  = 35.0
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

def left_dist(scan):
    idx = np.arange(85, 96) % 360
    return float(np.mean(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

def wall_follow(scan, fm, adir):
    ld = left_dist(scan)
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
    if ld > WALL_TARGET * 2.0:
        return (WALL_V * 0.7, WALL_LOST_W)

    err = ld - WALL_TARGET
    w = WALL_KP * err

    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.5
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V

    w = float(np.clip(w, -0.9, 0.9))
    return (v, w)

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 175], [179, 207, 255]),
               "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 214], [41, 143, 255]),
               "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 123], [138, 207, 246]),
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

# ── WALL FOLLOWING ─────────────────────────────────────────────────────
WALL_TARGET   = 25.0
WALL_KP       = 0.012
WALL_RANGE    = 20
WALL_V        = 0.22
WALL_TURN_V   = 0.10
WALL_LOST_W   = 0.4

# ── 도착 판정 영역 ─────────────────────────────────────────────────────
ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 40
ARRIVE_FORWARD_SEC = 1.0
ARRIVE_CONFIRM     = 8

# ── SEARCH 파라미터 (★ 신규) ──────────────────────────────────────────
# 1바퀴당 소요 시간(초) — 실측 후 조정. w=1.3rad/s 기준 약 2π/1.3 ≈ 4.8s
SPIN_ONE_ROUND_SEC  = 4.8
SPIN_ROUNDS         = 3          # 제자리 회전 바퀴 수
SPIN_W              = 1.3        # 제자리 회전 각속도
# 원형 탐색: 반경 ≈ v/w.  0.18/0.60 ≈ 0.30m = 30cm
CIRCLE_V            = 0.18
CIRCLE_W            = 0.60
CIRCLE_SEC          = 12.0       # 원형 탐색 최대 지속 시간(초)

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0
arrive_count  = 0

park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0
park_t        = None
last_cmd      = (0.0, 0.0)

# ★ SEARCH 세부 상태
search_sub      = "SPIN"   # "SPIN" | "CIRCLE"
search_t        = None     # 현재 search_sub 시작 시각

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
            err_x  = cx_obj - cx_mid
            last_seen_x   = cx_obj
            last_bottom_y = min(by_top + bh, 239)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)
            cv2.line(frame, (cx_obj, by_top), (cx_obj, by_top + bh), (0, 255, 255), 1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ══ LIDAR 모드 ═══════════════════════════════════════════
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

            v, w = wall_follow(scan, fm, adir)
            send_cmd(v, w)
            ld_disp = left_dist(scan)
            cv2.putText(frame, f"WALL-FOLLOW L:{ld_disp:.0f}cm", (10, 25),
                        0, 0.5, (255, 255, 255), 1)

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
                    if mission_idx < len(MISSION):
                        park_state  = "SEARCH"
                        search_sub  = "SPIN"        # ★ 다음 미션도 SPIN부터
                        search_t    = None
                        last_seen_x = cx_mid + 40
                        print(f"다음 미션 [{MISSION[mission_idx]}] 탐색 시작 (SPIN)")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # ── 3. 객체 추적 중 (TRACK) ────────────────────────────
            elif park_state == "TRACK" and found:
                # 찾았으면 search 상태 초기화
                search_sub = "SPIN"
                search_t   = None
                arrive_count = 0 if not centroid_in_arrive_zone() else arrive_count

                if centroid_in_arrive_zone():
                    arrive_count += 1
                else:
                    arrive_count = 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] centroid {ARRIVE_CONFIRM}프레임 확정 → {ARRIVE_FORWARD_SEC}초 전진")
                    send_cmd(*last_cmd)
                    continue
                else:
                    err_x = cx_obj - cx_mid

                    def cam_w(ex):
                        raw = -KP_ROT * ex
                        if abs(raw) < W_MIN and ex != 0:
                            return -W_MIN if ex > 0 else W_MIN
                        return raw

                    err_ratio = min(abs(err_x) / float(cx_mid), 1.0)
                    reduced_v = APPROACH_V * (1.0 - err_ratio)

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

            # ── 4. 탐색 (SEARCH) ★ 수정 핵심 부분 ────────────────
            else:
                park_state   = "SEARCH"
                arrive_count = 0

                now = time.time()

                # search_sub 진입 시각 기록
                if search_t is None:
                    search_t = now
                    print(f"[{target}] SEARCH 시작 → {search_sub}")

                elapsed_sub = now - search_t

                # 재발견 시 즉시 TRACK으로
                if found:
                    park_state = "TRACK"
                    search_sub = "SPIN"
                    search_t   = None
                    print(f"[{target}] 재발견 → TRACK")
                    continue

                # ── SPIN: 제자리 회전 ────────────────────────────
                if search_sub == "SPIN":
                    spin_total = SPIN_ONE_ROUND_SEC * SPIN_ROUNDS
                    if elapsed_sub < spin_total:
                        spin_dir = -1 if last_seen_x > cx_mid else 1
                        send_cmd(0.0, spin_dir * SPIN_W)
                        rounds_done = elapsed_sub / SPIN_ONE_ROUND_SEC
                        cv2.putText(frame,
                            f"SPIN {rounds_done:.1f}/{SPIN_ROUNDS} rev",
                            (10, 25), 0, 0.55, (0, 200, 255), 1)
                    else:
                        # 3바퀴 완료 → CIRCLE로 전환
                        search_sub = "CIRCLE"
                        search_t   = now
                        print(f"[{target}] SPIN 완료 → SEARCH_CIRCLE 시작")

                # ── CIRCLE: 원형 탐색 ────────────────────────────
                elif search_sub == "CIRCLE":
                    if elapsed_sub < CIRCLE_SEC:
                        # 장애물 감지 시 회피 우선
                        if fm < THRESH_TURN:
                            send_cmd(0.05, adir * 1.0)
                        else:
                            send_cmd(CIRCLE_V, CIRCLE_W)
                        cv2.putText(frame,
                            f"CIRCLE {elapsed_sub:.1f}/{CIRCLE_SEC:.0f}s",
                            (10, 25), 0, 0.55, (0, 140, 255), 1)
                    else:
                        # CIRCLE 시간 초과 → LIDAR 모드로 fallback
                        mode        = "LIDAR"
                        park_state  = "TRACK"
                        search_sub  = "SPIN"
                        search_t    = None
                        detect_count = 0
                        print(f"[{target}] CIRCLE 시간 초과 → LIDAR 복귀")

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
