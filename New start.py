import cv2
import serial
import numpy as np
import time
import math
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

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
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR START")

# ── LIDAR PARAMETERS ──────────────────────────────────────────────────
MAX_SPEED = 0.22
MIN_SPEED = 0.09
MAX_W = 0.9

THRESH_30 = 32.0
THRESH_20 = 22.0
THRESH_10 = 12.0

FRONT_CHECK_RANGE = 45

WALL_SEARCH_V = 0.15
WALL_SEARCH_W = 0.45
WALL_SEARCH_DIST = 80.0

# ── LIDAR FILTER ──────────────────────────────────────────────────────
EMA_ALPHA = 0.35
MEDIAN_K = 2
scan_data = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

# ── LIDAR UTIL (중복 함수 제거) ───────────────────────────────────────
def apply_ema(angle, new_dist_cm):
    if not isinstance(new_dist_cm, (int, float)) or new_dist_cm <= 0:
        return
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():
    k = MEDIAN_K
    window = 2*k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i+d)%360 for d in range(-k, k+1)]
        values = np.sort(scan_data[idx])
        filtered[i] = values[window//2]
    scan_data[:] = filtered

def get_front_min():
    idx = np.arange(-FRONT_CHECK_RANGE, FRONT_CHECK_RANGE+1) % 360
    return float(np.min(scan_data[idx]))

def choose_avoid_direction():
    left_avg = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    return 1 if left_avg >= right_avg else -1

def nearest_obstacle_angle():  # [수정] scan 매개변수 제거
    return int(np.argmin(scan_data))

def nearest_obstacle_dist():  # [수정] scan 매개변수 제거
    return float(np.min(scan_data))

def left_dist(scan):
    idx = np.arange(85, 96) % 360
    return float(np.mean(scan[idx]))

def side_min(scan, start, end):
    idx = np.arange(start, end) % 360
    return float(np.min(scan[idx]))

# ── LIDAR LOOP ────────────────────────────────────────────────────────
def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue
        
        s_flag = raw[0] & 0x01
        if ((raw[0]&0x02)>>1) != (1-s_flag) or (raw[1]&0x01)!=1 or (raw[0]>>2)<3:
            continue
        
        angle = int(((raw[1]>>1) | (raw[2]<<7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4]<<8)) / 40.0
        
        if 3 < dist_cm < 150:
            apply_ema(angle, dist_cm)
        
        if s_flag != 1:
            continue
        
        apply_median_filter()
        with scan_lock:
            scan_data[:] = scan_data

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock:
        return scan_data.copy()

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -MAX_SPEED, MAX_SPEED)
    w = np.clip(w, -MAX_W, MAX_W)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# ── WALL-FOLLOW 파라미터 및 함수 ──────────────────────────────────────
WALL_TARGET     = 30.0
WALL_SCAN_DIST  = 45.0
WALL_APPROACH_V = 0.18
WALL_KP         = 0.015
WALL_V          = 0.20
WALL_TURN_V     = 0.10
WALL_LOST_W     = 0.8

def wall_follow(scan, fm, adir):
    ld = left_dist(scan)
    left_close = side_min(scan, 60, 120)
    right_close = side_min(scan, 240, 300)

    if fm < THRESH_10:
        return (MIN_SPEED, adir * MAX_W)
    if fm < THRESH_20:
        return (0.12, adir * 0.8)

    if left_close < THRESH_10:
        return (WALL_V * 0.7, -0.9)
    if right_close < THRESH_10:
        return (WALL_V * 0.7,  0.9)

    if ld > WALL_TARGET * 2.0:
        nearest = nearest_obstacle_angle(scan)
        err_a = nearest if nearest <= 180 else nearest - 360
        w_recover = float(np.clip(-err_a / 90.0 * WALL_LOST_W, -WALL_LOST_W, WALL_LOST_W))
        return (WALL_V * 0.6, w_recover)

    err = ld - WALL_TARGET
    w = WALL_KP * err
    if fm < THRESH_30:
        blend = float(np.clip((THRESH_30 - fm) / (THRESH_30 - THRESH_20 + 1e-6), 0.0, 1.0))
        w = (1 - blend) * w + blend * adir * 0.7
        v = WALL_V * (1.0 - 0.4 * blend)
    else:
        v = WALL_V
    
    w = float(np.clip(w, -1.4, 1.4))
    return (v, w)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 136, 175], [179, 207, 255]),
               "hsv2": None,
               "bgr":  ([20, 20, 80], [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([24, 48, 193], [45, 170, 255]),
               "hsv2": None,
               "bgr":  ([0, 80, 80], [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([98, 100, 123], [138, 207, 246]),
               "hsv2": None,
               "bgr":  ([40, 0, 0], [255, 220, 220]), "draw": (255, 80, 0)},
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
MIN_AREA = 400
KP_ROT = 0.035
W_MIN = 0.30
APPROACH_V = 0.17
PARK_SEC = 1.2
DETECT_CONFIRM = 6

ARRIVE_Y_TOP = int(240 * 0.85)
ARRIVE_X_MARGIN = 40
ARRIVE_FORWARD_SEC = 0.7
ARRIVE_FORWARD_V = 0.15
ARRIVE_CONFIRM = 8

FULL_CIRCLE_RAD = 2 * math.pi * 0.85
ESCAPE_RAD = math.pi * 0.6
ESCAPE_V = 0.13
ESCAPE_W = 1.70

# ── STATE ─────────────────────────────────────────────────────────────
mode = "LIDAR"
mission_idx = 0
detect_count = 0
arrive_count = 0

park_state = "TRACK"
last_seen_x = 160
last_bottom_y = 0
park_t = None
last_cmd = (0.0, 0.0)

wf_angle_accum = 0.0
wf_last_t = None
esc_angle_accum = 0.0
ws_start_t = None

print(f"START | MISSION: {MISSION}")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]
        cx_mid = W // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        scan = get_scan()
        fm = get_front_min()
        adir = choose_avoid_direction()

        # ── [EMERGENCY] 사방 폐쇄 탈출 ────────────────────────────────
        left_avg = float(np.mean(scan[45:135]))
        right_avg = float(np.mean(scan[225:315]))
        
        if fm < 35.0 and left_avg < 35.0 and right_avg < 35.0:
            print("[EMERGENCY] 사방이 벽으로 가로막힘! 180 도 회전")
            stop_robot()
            time.sleep(0.1)
            send_cmd(0.0, adir * MAX_W)
            time.sleep(0.75)
            stop_robot()
            time.sleep(0.1)
            continue

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw = COLOR_CFG[target]["draw"]

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
            last_seen_x = cx_obj
            last_bottom_y = min(by_top + bh, 239)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.circle(frame, (cx_obj, cy_obj), 5, (0, 255, 255), -1)
            cv2.line(frame, (cx_obj, by_top), (cx_obj, by_top + bh), (0, 255, 255), 1)

        arrive_x1 = cx_mid - ARRIVE_X_MARGIN
        arrive_x2 = cx_mid + ARRIVE_X_MARGIN
        cv2.rectangle(frame, (arrive_x1, ARRIVE_Y_TOP), (arrive_x2, H - 1), (0, 0, 255), 1)

        def centroid_in_arrive_zone():
            return (cx_obj >= arrive_x1 and cx_obj <= arrive_x2 and cy_obj >= ARRIVE_Y_TOP)

        # ══ LIDAR 모드 ════════════════════════════════════════════════
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

            nearest = nearest_obstacle_angle()  # [수정] 매개변수 없이 호출
            nd = nearest_obstacle_dist()  # [수정] 매개변수 없이 호출
            
            if nd < WALL_SEARCH_DIST:
                err_a = nearest if nearest <= 180 else nearest - 360
                
                if abs(err_a) > 10:
                    w_s = float(np.clip(-err_a / 60.0 * WALL_SEARCH_W, -WALL_SEARCH_W, WALL_SEARCH_W))
                    v_s = WALL_SEARCH_V
                else:
                    v_s = WALL_SEARCH_V * 1.2
                    w_s = 0.0
                
                if fm < THRESH_10:
                    v_s = MIN_SPEED
                    w_s = adir * MAX_W
                    print(f"[ VERY CLOSE] {fm:.1f}cm - 급회전 회피")
                elif fm < THRESH_20:
                    v_s = 0.12
                    w_s = adir * 0.8
                    print(f"[CRITICAL] {fm:.1f}cm - 회피")
                elif fm < THRESH_30:
                    v_s = 0.15
                    w_s = adir * 0.7
                    print(f"[WARNING] {fm:.1f}cm - 감속")
                
                send_cmd(v_s, w_s)
                cv2.putText(frame, f"LIDAR: 장애물 탐색 nd={nd:.0f}cm", (10, 25), 0, 0.5, (0, 255, 0), 1)
            else:
                v = WALL_SEARCH_V
                w = adir * 0.2
                
                if fm < THRESH_10:
                    v, w = MIN_SPEED, adir * MAX_W
                    print(f"[ VERY CLOSE] {fm:.1f}cm - 급회전 회피")
                elif fm < THRESH_20:
                    v, w = 0.12, adir * 0.8
                    print(f"[CRITICAL] {fm:.1f}cm - 회피")
                elif fm < THRESH_30:
                    v, w = 0.15, adir * 0.7
                    print(f"[WARNING] {fm:.1f}cm - 감속")
                
                send_cmd(v, w)
                cv2.putText(frame, f"LIDAR: 주변 탐색 fm={fm:.0f}cm", (10, 25), 0, 0.5, (0, 255, 0), 1)

        # ══ PARK 모드 ════════════════════════════════════════════════
        elif mode == "PARK":
            if park_state in ("WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW",
                              "WALL_ESCAPE", "SEARCH"):
                if found:
                    detect_count += 1
                    if detect_count >= DETECT_CONFIRM:
                        detect_count = 0
                        park_state = "TRACK"
                        wf_angle_accum = 0.0
                        wf_last_t = None
                        esc_angle_accum = 0.0
                        ws_start_t = None
                        print(f"[{target}] 탐색 중 발견 → TRACK")
                        continue
                else:
                    detect_count = 0

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

            elif park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    arrive_count = 0
                    detect_count = 0
                    wf_angle_accum = 0.0
                    wf_last_t = None
                    esc_angle_accum = 0.0
                    if mission_idx < len(MISSION):
                        park_state = "WALL_SEARCH"
                        ws_start_t = time.time()
                        print(f"다음 미션 [{MISSION[mission_idx]}] wall-following 탐색 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            elif park_state == "WALL_SEARCH":
                if ws_start_t is None:
                    ws_start_t = time.time()

                front_nearest = side_min(scan, 315, 405)
                if front_nearest < WALL_SCAN_DIST:
                    park_state = "WALL_APPROACH"
                    ws_start_t = None
                    print(f"장애물 감지 {front_nearest:.0f}cm → 접근 시작")
                    continue

                nearest_sa = nearest_obstacle_angle(scan)
                nd_sa = nearest_obstacle_dist(scan)

                if nd_sa < 80.0:
                    err_a = nearest_sa if nearest_sa <= 180 else nearest_sa - 360
                    w_s = float(np.clip(-err_a / 90.0 * 1.2, -1.2, 1.2))
                    send_cmd(0.0, w_s)
                else:
                    send_cmd(0.15, 0.45)

                if time.time() - ws_start_t > 6.0:
                    print("[개활지 예외 처리] 장시간 장애물 미검출 -> 강제 직진")
                    send_cmd(0.22, 0.0)
                    time.sleep(0.8)
                    ws_start_t = time.time()

                cv2.putText(frame, f"WALL-SEARCH [{target}] nd={nd_sa:.0f}cm",
                            (10, 25), 0, 0.5, (0, 255, 0), 1)

            elif park_state == "WALL_APPROACH":
                nd = nearest_obstacle_dist(scan)
                ld = left_dist(scan)

                if nd <= WALL_TARGET * 1.3:
                    park_state = "WALL_FOLLOW"
                    wf_angle_accum = 0.0
                    wf_last_t = time.time()
                    esc_angle_accum = 0.0
                    print(f"장애물 도달 {nd:.0f}cm → wall-following 시작")
                    continue

                if fm < THRESH_10:
                    v, w = MIN_SPEED, adir * MAX_W
                elif fm < THRESH_20:
                    v, w = WALL_APPROACH_V * 0.6, adir * 0.9
                else:
                    nearest_wa = nearest_obstacle_angle(scan)
                    err_a = nearest_wa if nearest_wa <= 180 else nearest_wa - 360
                    w_a = float(np.clip(-err_a / 120.0 * 0.6, -0.6, 0.6))
                    v, w = WALL_APPROACH_V, w_a
                send_cmd(v, w)
                cv2.putText(frame, f"WALL-APPROACH [{target}] nd={nd:.0f}cm",
                            (10, 25), 0, 0.45, (0, 200, 0), 1)

            elif park_state == "WALL_FOLLOW":
                now = time.time()
                dt = now - wf_last_t if wf_last_t else 0.0
                wf_last_t = now

                v, w = wall_follow(scan, fm, adir)
                send_cmd(v, w)

                wf_angle_accum += abs(w) * dt

                if wf_angle_accum >= FULL_CIRCLE_RAD:
                    park_state = "WALL_ESCAPE"
                    esc_angle_accum = 0.0
                    wf_angle_accum = 0.0
                    wf_last_t = None
                    print(f"[{target}] 순환 감지 → 이탈 시작")

                ld_disp = left_dist(scan)
                accum_deg = math.degrees(wf_angle_accum)
                cv2.putText(frame,
                            f"WALL-FOLLOW [{target}] L:{ld_disp:.0f}cm rot:{accum_deg:.0f}deg",
                            (10, 25), 0, 0.45, (0, 255, 0), 1)

            elif park_state == "WALL_ESCAPE":
                now = time.time()
                dt = now - wf_last_t if wf_last_t else 0.0
                wf_last_t = now

                send_cmd(ESCAPE_V, -ESCAPE_W)
                esc_angle_accum += ESCAPE_W * dt

                if esc_angle_accum >= ESCAPE_RAD:
                    park_state = "WALL_SEARCH"
                    ws_start_t = time.time()
                    esc_angle_accum = 0.0
                    wf_last_t = None
                    wf_angle_accum = 0.0
                    print(f"[{target}] 이탈 완료 → WALL_SEARCH")

                cv2.putText(frame, f"WALL-ESCAPE [{target}]",
                            (10, 25), 0, 0.5, (0, 128, 255), 1)

            elif found:
                park_state = "TRACK"
                wf_angle_accum = 0.0
                wf_last_t = None
                esc_angle_accum = 0.0
                ws_start_t = None
                arrive_count = arrive_count + 1 if centroid_in_arrive_zone() else 0

                if arrive_count >= ARRIVE_CONFIRM:
                    arrive_count = 0
                    park_state = "FORWARD"
                    park_t = time.time()
                    print(f"[{target}] centroid {ARRIVE_CONFIRM}프레임 확정 → 전진")
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

                    if fm >= THRESH_30:
                        v = reduced_v
                        w = cam_w(err_x)
                    else:
                        w_cam = cam_w(err_x)
                        w_lid = adir * 0.9
                        if fm < THRESH_10:
                            v, w = MIN_SPEED, w_lid
                        elif fm < THRESH_20:
                            v, w = 0.12, 0.7 * w_lid + 0.3 * w_cam
                        else:
                            v, w = reduced_v, 0.3 * w_lid + 0.7 * w_cam

                    last_cmd = (v, w)
                    send_cmd(v, w)

                cv2.putText(frame, f"TRACKING: {target}", (10, 25), 0, 0.6, draw, 1)

            else:
                if park_state != "SEARCH":
                    park_state = "SEARCH"
                    arrive_count = 0
                    print(f"[{target}] 객체 놓침 → SEARCH")

                w = -1.8 if last_seen_x > cx_mid else 1.8
                send_cmd(0.0, w)

                if park_t is None or park_state != "SEARCH":
                    park_t = time.time()
                if time.time() - park_t > 2.5:
                    park_state = "WALL_SEARCH"
                    ws_start_t = time.time()
                    park_t = None
                    wf_angle_accum = 0.0
                    wf_last_t = None
                    print(f"[{target}] SEARCH 2.5 초 초과 → WALL_SEARCH")

                cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    lidar_ser.close()
    cv2.destroyAllWindows()
