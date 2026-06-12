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
FRONT_RANGE  = 50  
THRESH_SLOW  = 55.0  
THRESH_TURN  = 35.0  
THRESH_STOP  = 8.0  

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

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6) 
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
KP_ROT         = 0.0035  # 부드러운 회전을 위한 조향 감도 조정
APPROACH_V     = 0.18    # 정확한 안착을 위해 전진 속도를 안정적으로 하향
PARK_SEC       = 1.5     # 정차 유지 시간
DETECT_CONFIRM = 3 
BOTTOM_10PCT   = int(240 * 0.90)  

LEFT_20PCT     = int(320 * 0.20)  
RIGHT_20PCT    = int(320 * 0.80)  

# 탐색 실패 시 시퀀스 파라미터
ROTATION_2_TURNS_TIME = 4.5  
MOVE_5CM_TIME         = 0.25 

# 🚨 [신규] 사각지대 진입 시 마지막 메모리 주행 파라미터
BLIND_PARK_SEC        = 0.65 # 카메라에서 사라진 뒤 마지막 오차 방향으로 돌며 정인입할 시간 (초)

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "START_SEARCH"  
mission_idx   = 0
detect_count  = 0

park_state    = "SEARCH" 
last_seen_x   = 160
last_err_x    = 0        # 🎯 마지막으로 기억한 추적 오차값
was_in_bottom = False
was_in_left   = False
was_in_right  = False

park_t        = None
blind_start_t = None     # 사각지대 진입 타임스탬프

search_sub_state = "ROTATE"  
search_start_t   = time.time()

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

        # 가이드라인 시각화
        cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)
        cv2.line(frame, (LEFT_20PCT, 0), (LEFT_20PCT, H), (255, 0, 0), 1)   
        cv2.line(frame, (RIGHT_20PCT, 0), (RIGHT_20PCT, H), (255, 0, 0), 1) 

        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox     = bx + bw // 2
            by_bot = min(by_top + bh, 239)
            err_x  = ox - cx_mid
            
            # 🎯 실시간 중심점 데이터 및 오차 갱신 저장
            last_seen_x   = ox
            last_err_x    = err_x 
            
            # 색지가 하단 영역에 도달했고 + 정면 범위 내에 안착했는지 기록
            was_in_bottom = (by_bot >= BOTTOM_10PCT) and (LEFT_20PCT < ox < RIGHT_20PCT)
            was_in_left   = (ox <= LEFT_20PCT)
            was_in_right  = (ox >= RIGHT_20PCT)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)

        # ══ START_SEARCH 모드 ═════════════════════════════════════════
        if mode == "START_SEARCH":
            if found:
                detect_count += 1
                if detect_count >= DETECT_CONFIRM:
                    detect_count = 0
                    mode = "PARK"
                    park_state = "TRACK"
                    print(f" 정면 즉시 포착 완료! [{target}] 미션 진입.")
                    continue
                v, w = 0.0, 0.0 
            else:
                detect_count = 0
                v, w = 0.0, 1.3 

            send_cmd(v, w)
            cv2.putText(frame, "MODE: START_SEARCH", (10, 25), 0, 0.5, (0, 255, 255), 1)

        # ══ LIDAR 모드 (장애물 제자리 조향 회피 제어) ══════════════════════
        elif mode == "LIDAR":
            if fm >= THRESH_SLOW:
                print(" 장애물 회피 완료 -> 다시 객체 탐색 시퀀스 전환")
                mode = "PARK"
                park_state = "SEARCH" 
                search_sub_state = "ROTATE"
                search_start_t = time.time()
                was_in_bottom = was_in_left = was_in_right = False
                continue

            v = 0.0
            w = adir * 1.3
            send_cmd(v, w)
            cv2.putText(frame, "MODE: LIDAR (PIVOT AVOID)", (10, 25), 0, 0.5, (0, 0, 255), 1)

        # ══ PARK 모드 (추적 / 메모리 주행 / 정차 / 탐색) ═══════════════════
        elif mode == "PARK":
            if fm < THRESH_SLOW and park_state != "PARKING":
                print("⚠️ 장애물 감지! 제자리 회피(LIDAR) 모드로 전환합니다.")
                mode = "LIDAR"
                continue

            # 1. 완벽 정차 중 (PARKING)
            if park_state == "PARKING":
                stop_robot()
                if time.time() - park_t >= PARK_SEC:
                    mission_idx += 1
                    if mission_idx < len(MISSION):
                        park_state = "SEARCH"
                        search_sub_state = "ROTATE"
                        search_start_t = time.time()
                        last_seen_x = cx_mid + 40 
                        last_err_x = 0
                        was_in_bottom = was_in_left = was_in_right = False
                        print(f"다음 미션 [{MISSION[mission_idx]}] 탐색 회전 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # 2. 🎯 [신규 변환] 메모리 기반 최종 진입 중 (BLIND_ENTRY)
            elif park_state == "BLIND_ENTRY":
                # 카메라엔 안 보이지만, 마지막 오차(last_err_x)를 보정하는 조향을 유지하며 최종 직진 전진
                w_cam = -KP_ROT * last_err_x
                v, w = APPROACH_V, w_cam
                
                send_cmd(v, w)
                cv2.putText(frame, f"BLIND ENTRY: LAST ERR {last_err_x:.1f}", (10, 25), 0, 0.6, (255, 0, 255), 2)
                
                # 지정된 시간만큼 밀고 들어갔다면 완전 정차(PARKING)로 전환
                if time.time() - blind_start_t >= BLIND_PARK_SEC:
                    print(f"[{target}] 최종 메모리 주행 완료 -> 완벽 안착 판정!")
                    park_state = "PARKING"
                    park_t = time.time()
                continue

            # 3. 객체 추적 중 (TRACK)
            elif found:
                park_state = "TRACK"
                w_cam = -KP_ROT * err_x
                
                # 중심점 정렬 상태에 따라 속도를 가감하여 오버슈팅 방지
                v_adj = APPROACH_V * max(0.5, (1.0 - abs(err_x) / cx_mid))
                v, w = v_adj, w_cam
                
                send_cmd(v, w)
                cv2.putText(frame, f"TRACKING: {target} | Err: {err_x}", (10, 25), 0, 0.6, draw, 1)

            # 4. 객체 놓침 또는 다음 객체 탐색 (SEARCH)
            else:
                # 🚨 정면 바닥까지 잘 정렬되어 들어오다가 시야에서 '방금 막' 사라진 상황 판정!
                if was_in_bottom and not was_in_left and not was_in_right:
                    print(f"🎯 중심점 락온 상태에서 사각지대 진입 -> BLIND_ENTRY 전환 (기억된 오차: {last_err_x})")
                    park_state = "BLIND_ENTRY"
                    blind_start_t = time.time()
                    was_in_bottom = was_in_left = was_in_right = False
                    continue
                
                # 정면 진입 실패 후 완전히 놓친 일반 탐색 상태
                park_state = "SEARCH"
                now = time.time()
                
                if was_in_left:
                    v, w = 0.0, 1.6  
                    cv2.putText(frame, "SNAP TURN: LEFT ESCAPE", (10, 50), 0, 0.5, (0, 0, 255), 2)
                    search_start_t = now
                elif was_in_right:
                    v, w = 0.0, -1.6 
                    cv2.putText(frame, "SNAP TURN: RIGHT ESCAPE", (10, 50), 0, 0.5, (0, 0, 255), 2)
                    search_start_t = now
                else:
                    # 2바퀴 회전 후 5cm 전진 로직
                    if search_sub_state == "ROTATE":
                        v = 0.0
                        w = -1.3 if last_seen_x > cx_mid else 1.3
                        if now - search_start_t >= ROTATION_2_TURNS_TIME:
                            search_sub_state = "GO_FORWARD"
                            search_start_t = now
                            
                    elif search_sub_state == "GO_FORWARD":
                        v = 0.2
                        w = 0.0
                        if now - search_start_t >= MOVE_5CM_TIME:
                            search_sub_state = "ROTATE"
                            search_start_t = now
                
                was_in_bottom = False 
                send_cmd(v, w)
                cv2.putText(frame, f"SEARCH ({search_sub_state}): {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
