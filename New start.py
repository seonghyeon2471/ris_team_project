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

# ── LIDAR PARAMS ──────────────────────────────────────────────────────
EMA_ALPHA    = 0.35
MEDIAN_K     = 2
FRONT_RANGE  = 90

THRESH_SLOW  = 40.0
THRESH_TURN  = 24.0
THRESH_STOP  = 12.0

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
        if len(raw) != 5:
            continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150:
            _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock:
                _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

print("WAITING FOR FIRST SCAN DATA...")
time.sleep(1.5)

def get_scan():
    with scan_lock:
        return _scan_pub.copy()

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

def decide_follow_side(adir, found, cx_obj, cx_mid):
    if found and cx_obj >= 0:
        return "L" if cx_obj < cx_mid else "R"
    return "R" if adir == 1 else "L"

def side_dist(scan, side):
    if side == "L":
        idx = np.arange(265, 296) % 360
    else:
        idx = np.arange(65, 96) % 360
    return float(np.min(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

def wall_follow(scan, fm, adir, follow_side, found, last_seen_x, cx_mid):
    sd          = side_dist(scan, follow_side)
    left_close  = side_min(scan, 240, 300)
    right_close = side_min(scan, 60, 120)
    sign = 1 if follow_side == "L" else -1

    if not found:
        camera_dir = 1.0 if last_seen_x < cx_mid else -1.0
        adir = camera_dir

    if fm < THRESH_STOP:
        return (0.02, adir * 1.5)
    if fm < THRESH_TURN:
        return (WALL_TURN_V, adir * 1.45)

    if left_close < SIDE_STOP:
        return (WALL_V * 0.4, -1.2)
    if right_close < SIDE_STOP:
        return (WALL_V * 0.4, 1.2)

    if left_close < BOTTLENECK_ENTER and right_close < BOTTLENECK_ENTER:
        err_center = left_close - right_close
        w_center = float(np.clip(CENTER_KP * err_center, -0.9, 0.9))
        return (BOTTLENECK_V, w_center)

    if sd > WALL_TARGET * 2.0:
        return (0.05, sign * WALL_LOST_W)

    err = sd - WALL_TARGET
    w = sign * WALL_KP * err

    if fm < THRESH_SLOW:
        blend = float(np.clip((THRESH_SLOW - fm) / (THRESH_SLOW - THRESH_TURN + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 1.25
        v = WALL_V * (1.0 - 0.70 * blend)
    else:
        v = WALL_V

    if not found and fm >= THRESH_SLOW:
        is_safe = True
        if adir > 0 and left_close < (WALL_TARGET * 1.3):
            is_safe = False
        if adir < 0 and right_close < (WALL_TARGET * 1.3):
            is_safe = False
        if is_safe:
            w += (adir * 0.20)

    w = float(np.clip(w, -1.5, 1.5))
    return (v, w)

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 114], [179, 220, 255]), "hsv2": None, "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]),    "hsv2": None, "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 95], [138, 207, 246]),   "hsv2": None, "bgr":  ([40, 0, 0],    [255, 220, 220]), "draw": (255, 80, 0)},
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

# ── WALL FOLLOW / PARK 세부 파라미터 ───────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.030
W_MIN          = 0.20
APPROACH_V     = 0.13
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP       = int(240 * 0.85)
ARRIVE_X_MARGIN    = 30
ARRIVE_FORWARD_SEC = 0.8
ARRIVE_FORWARD_V   = 0.13
ARRIVE_CONFIRM     = 8

WALL_TARGET      = 12.0
SIDE_STOP        = 9.5
BOTTLENECK_ENTER = 15.0
CENTER_KP        = 0.050
BOTTLENECK_V     = 0.11
WALL_SCAN_DIST   = 80.0
SIDE_SCAN_DIST   = 22.0

WALL_APPROACH_V  = 0.14
WALL_KP          = 0.030
WALL_V           = 0.17

WALL_TURN_V      = 0.04
WALL_LOST_W      = 1.5
WALL_SEARCH_W    = 1.1
MISSION_TIMEOUT_SEC = 10.0

# 개활지 탐색 추가
SEARCH_FORWARD_DIST_SEC = 2.5
SEARCH_FORWARD_V        = 0.10
SEARCH_ROTATE_W         = 1.2
SEARCH_ROTATE_SEC       = 5.4

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
hop_start_t      = None
last_cmd      = (0.0, 0.0)

search_phase  = "FORWARD"
search_phase_t = time.time()

print(f"START | MISSION: {MISSION}")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

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
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

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
        big = max(cnts, key=cv2.contourArea) if cnts else None
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

        if mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                continue

            if lidar_state == "WALL_SEARCH":
                left_close  = side_min(scan, 240, 300)
                right_close = side_min(scan, 60, 120)

                if fm < WALL_SCAN_DIST or left_close < SIDE_SCAN_DIST or right_close < SIDE_SCAN_DIST:
                    follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                    lidar_state = "WALL_APPROACH"
                    search_phase = "FORWARD"
                    search_phase_t = time.time()
                else:
                    if search_phase == "FORWARD":
                        if time.time() - search_phase_t < SEARCH_FORWARD_DIST_SEC:
                            send_cmd(SEARCH_FORWARD_V, 0.0)
                        else:
                            search_phase = "ROTATE"
                            search_phase_t = time.time()
                    elif search_phase == "ROTATE":
                        if time.time() - search_phase_t < SEARCH_ROTATE_SEC:
                            send_cmd(0.0, SEARCH_ROTATE_W)
                        else:
                            search_phase = "FORWARD"
                            search_phase_t = time.time()

            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    lidar_state = "WALL_FOLLOW"
                else:
                    sign = 1 if follow_side == "L" else -1
                    if fm < THRESH_STOP:
                        send_cmd(0.02, adir * 1.4)
                    elif fm < THRESH_TURN:
                        send_cmd(WALL_APPROACH_V * 0.4, adir * 1.1)
                    else:
                        send_cmd(WALL_APPROACH_V, sign * 0.3)

            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side, found, last_seen_x, cx_mid)
                send_cmd(v, w)

        elif mode == "PARK":
            if park_state == "SAFE_HOP":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        mission_start_t = time.time()
                        continue
                else:
                    detect_count = 0

                elapsed_hop = time.time() - hop_start_t

                if elapsed_hop < 2.0:
                    send_cmd(0.0, 1.2)
                else:
                    if fm > 100.0:
                        send_cmd(0.0, 1.0)
                    elif fm > 40.0:
                        send_cmd(WALL_V, 0.0)
                    else:
                        follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                        park_state = "WALL_APPROACH"
                        mission_start_t = time.time()
                        continue

                cv2.imshow("f", frame)
                cv2.waitKey(1)
                continue

            if park_state == "FORWARD":
                elapsed = time.time() - park_t
                if elapsed >= ARRIVE_FORWARD_SEC:
                    stop_robot()
                    park_state = "PARKING"
                    park_t = time.time()
                else:
                    send_cmd(*last_cmd)

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
                else:
                    detect_count = 0

                left_close  = side_min(scan, 240, 300)
                right_close = side_min(scan, 60, 120)

                if fm < WALL_SCAN_DIST or left_close < SIDE_SCAN_DIST or right_close < SIDE_SCAN_DIST:
                    follow_side = decide_follow_side(adir, found, cx_obj, cx_mid)
                    park_state = "WALL_APPROACH"
                    continue

                if search_phase == "FORWARD":
                    if time.time() - search_phase_t < SEARCH_FORWARD_DIST_SEC:
                        send_cmd(SEARCH_FORWARD_V, 0.0)
                    else:
                        search_phase = "ROTATE"
                        search_phase_t = time.time()
                elif search_phase == "ROTATE":
                    if time.time() - search_phase_t < SEARCH_ROTATE_SEC:
                        send_cmd(0.0, SEARCH_ROTATE_W)
                    else:
                        search_phase = "FORWARD"
                        search_phase_t = time.time()

            elif park_state == "WALL_APPROACH":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        continue
                else:
                    detect_count = 0

                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    park_state = "WALL_FOLLOW"
                    continue

                sign = 1 if follow_side == "L" else -1
                if fm < THRESH_STOP:
                    v, w = 0.02, adir * 1.4
                elif fm < THRESH_TURN:
                    v, w = WALL_APPROACH_V * 0.4, adir * 1.1
                else:
                    v, w = WALL_APPROACH_V, sign * 0.3
                send_cmd(v, w)

            elif park_state == "WALL_FOLLOW":
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        continue
                else:
                    detect_count = 0

                v, w = wall_follow(scan, fm, adir, follow_side, found, last_seen_x, cx_mid)
                send_cmd(v, w)

            elif park_state == "TRACK":
                if not found:
                    park_state = "SEARCH"
                    search_t = time.time()
                    arrive_count = 0
                    continue

                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
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
                        v, w = reduced_v, cam_w(err_x)
                    else:
                        w_cam = cam_w(err_x)
                        w_lid = adir * 0.75
                        if fm < THRESH_STOP:
                            v, w = 0.02, w_lid * 1.5
                        elif fm < THRESH_TURN:
                            v, w = 0.04, 0.8 * (w_lid * 1.45) + 0.2 * w_cam
                        else:
                            v, w = reduced_v, 0.4 * w_lid + 0.6 * w_cam

                    last_cmd = (v, w)
                    send_cmd(v, w)

            elif park_state == "SEARCH":
                elapsed_search = time.time() - search_t
                if elapsed_search > 5.0:
                    park_state = "WALL_SEARCH"
                    search_phase = "FORWARD"
                    search_phase_t = time.time()
                else:
                    v = 0.0
                    w = (-1.0 if last_seen_x > cx_mid else 1.0)
                    send_cmd(v, w)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
