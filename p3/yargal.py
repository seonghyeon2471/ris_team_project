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
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
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

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

# ── MOTOR (각속도 제한 대폭 해제) ───────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -2.8, 2.8)  # [수정] 기존 1.6에서 2.8로 상한선 크게 확장
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red":    {"hsv1": ([169, 168, 96], [179, 222, 157]),
               "hsv2": None,
               "bgr":  ([20, 20, 80],  [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([16, 137, 142], [30, 214, 195]),
               "hsv2": None,
               "bgr":  ([0, 80, 80],   [255, 255, 255]), "draw": (0, 200, 255)},
    "blue":   {"hsv1": ([106, 168, 54], [131, 210, 82]),
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
MAX_W_TRACK = 0.5
DEADZONE = 0.05
APPROACH_V     = 0.22
PARK_SEC       = 1.2
DETECT_CONFIRM = 6

# [바운더리 파라미터] 좌우 외곽 10% 정의
BOTTOM_10PCT   = int(240 * 0.90)  # 216px
LEFT_10PCT     = int(320 * 0.10)  # 32px
RIGHT_10PCT    = int(320 * 0.90)  # 288px

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0

# PARK 세부 상태
park_state    = "TRACK"
last_seen_x   = 160
last_bottom_y = 0

# 경계 상태 플래그
was_in_bottom = False
was_in_left   = False
was_in_right  = False

park_t        = None

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

        # 디버깅용 가이드라인 선 그리기
        cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)
        cv2.line(frame, (LEFT_10PCT, 0), (LEFT_10PCT, H), (255, 0, 0), 1)
        cv2.line(frame, (RIGHT_10PCT, 0), (RIGHT_10PCT, H), (255, 0, 0), 1)

        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox     = bx + bw // 2
            by_bot = min(by_top + bh, 239)
            err_x = ox - cx_mid

            # -1 ~ +1 정규화
            err_norm = err_x / (W / 2)

            # 중앙 근처 흔들림 제거
            if abs(err_norm) < DEADZONE:
                err_norm = 0.0

            # 센트로이드 오차에 비례한 회전량
            w_track = -MAX_W_TRACK * err_norm
            
            last_seen_x   = ox
            last_bottom_y = by_bot
            
            # 실시간 이탈 경계면 저장
            was_in_bottom = (by_bot >= BOTTOM_10PCT)
            was_in_left   = (bx <= LEFT_10PCT)
            was_in_right  = ((bx + bw) >= RIGHT_10PCT)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)

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

            if fm < THRESH_STOP: v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN: v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW: v, w = 0.18, adir * 0.4
            else: v, w = 0.28, 0.0
            send_cmd(v, w)
            cv2.putText(frame, "MODE: LIDAR", (10, 25), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드 (추적 / 정차 / 탐색) ══════════════════════════
        elif mode == "PARK":
            # 1. 정차 중 (PARKING)
            if park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    if mission_idx < len(MISSION):
                        park_state = "SEARCH"
                        # 주차 후 우회전 탐색 속도도 더 빠르게 기동 시퀀스로 연결되도록 유도
                        last_seen_x = cx_mid + 40 
                        was_in_bottom = was_in_left = was_in_right = False 
                        print(f"다음 미션 [{MISSION[mission_idx]}] 탐색 회전 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # 2. 객체 추적 중 (TRACK)
            elif found:
                park_state = "TRACK"
                if fm >= THRESH_SLOW:

                    v = APPROACH_V
                    w = w_track

                else:

                    w_cam = w_track
                    w_lid = adir * 0.7

                    if fm < THRESH_STOP:

                        v = 0.09
                        w = w_lid

                    elif fm < THRESH_TURN:

                        v = 0.13
                        w = 0.7 * w_lid + 0.3 * w_cam

                    else:

                        v = 0.18
                        w = 0.3 * w_lid + 0.7 * w_cam
                send_cmd(v, w)
                cv2.putText(frame, f"TRACKING: {target}", (10, 25), 0, 0.6, draw, 1)

            # 3. 객체 놓침 또는 다음 객체 탐색 (SEARCH)
            else:
                if was_in_bottom:
                    park_state = "PARKING"
                    park_t = time.time()
                    was_in_bottom = was_in_left = was_in_right = False
                    print(f"[{target}] 전방 하단 도착 판정 → 주차")
                
                else:
                    park_state = "SEARCH"
                    v = 0.0
                    
                    # [강화된 초고속 바운더리 복귀 조향]
                    if was_in_left:
                        w = 2.50  # [수정] 기존 1.60 -> 2.50으로 제자리 좌회전 대폭 강화
                        cv2.putText(frame, "ESCAPE: LEFT SIDE! FAST SNAP TURN", (10, 50), 0, 0.5, (0, 0, 255), 2)
                    elif was_in_right:
                        w = -2.50 # [수정] 기존 -1.60 -> -2.50으로 제자리 우회전 대폭 강화
                        cv2.putText(frame, "ESCAPE: RIGHT SIDE! FAST SNAP TURN", (10, 50), 0, 0.5, (0, 0, 255), 2)
                    else:
                        # 주차 직후 다음 타겟 탐색 등 일반 SEARCH 상태 회전 속도도 상향
                        w = -1.8 if last_seen_x > cx_mid else 1.8  # [수정] 기존 1.3 -> 1.8
                    
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
