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
KP_ROT         = 0.004   # 정밀 조향을 위해 약간 상향 조정 (중심 맞추기 강화)
APPROACH_V     = 0.20   # 주차 안정성을 위해 접근 속도 살짝 하향 (0.22 -> 0.20)
PARK_SEC       = 1.2
DETECT_CONFIRM = 3 
BOTTOM_10PCT   = int(240 * 0.90)  

LEFT_20PCT     = int(320 * 0.20)  
RIGHT_20PCT    = int(320 * 0.80)  

# 🔄 탐색 실패 시 전진 시퀀스 파라미터
ROTATION_2_TURNS_TIME = 4.5  # 각속도 1.3 기준 2바퀴 도는데 걸리는 추정 시간 (초)
MOVE_5CM_TIME         = 0.25 # 속도 0.2로 약 5cm 이동하는 데 걸리는 시간 (초)

# ── STATE ─────────────────────────────────────────────────────────────
mode          = "START_SEARCH"  
mission_idx   = 0
detect_count  = 0

park_state    = "SEARCH" # 초기 SEARCH 상태로 수정
last_seen_x   = 160
last_bottom_y = 0
was_in_bottom = False
was_in_left   = False
was_in_right  = False

park_t        = None

# 탐색 제어 변수
search_sub_state = "ROTATE"  # "ROTATE" 또는 "GO_FORWARD"
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
            last_seen_x   = ox
            last_bottom_y = by_bot
            
            # 정면 구역(좌우 20% 사이) 및 중심점 정렬이 잘 되어있는지 확인
            # 조향 오차 정밀도 조건 추가 abs(err_x) < 25 (정면 중심에 가깝게 정렬되었는가?)
            was_in_bottom = (by_bot >= BOTTOM_10PCT) and (LEFT_20PCT < ox < RIGHT_20PCT) and (abs(err_x) < 25)
            
            was_in_left   = (ox <= LEFT_20PCT)
            was_in_right  = (ox >= RIGHT_20PCT)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)

        # ══ START_SEARCH 모드 (시작 시 제자리 회전 탐색) ══════════════
        if mode == "START_SEARCH":
            if found:
                detect_count += 1
                if detect_count >= DETECT_CONFIRM:
                    detect_count = 0
                    mode = "PARK"
                    park_state = "TRACK"
                    print(f" 정면 즉시 포착 완료! [{target}] 미션 바로 진입.")
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
                search_start_t = time.time() # 타임아웃 초기화
                was_in_bottom = was_in_left = was_in_right = False
                continue

            # 전진 속도는 0.0으로 묶고, 제자리에서 안전 방향으로만 회전
            v = 0.0
            w = adir * 1.3
            
            send_cmd(v, w)
            cv2.putText(frame, "MODE: LIDAR (PIVOT AVOID)", (10, 25), 0, 0.5, (0, 0, 255), 1)

        # ══ PARK 모드 (추적 / 정차 / 탐색) ══════════════════════════
        elif mode == "PARK":
            # 주행(TRACK) 또는 탐색전진 중 장애물이 슬로우 영역(55cm) 안으로 들어오면 회피모드로 변경
            if fm < THRESH_SLOW:
                print("⚠️ 장애물 감지! 제자리 회피(LIDAR) 모드로 전환합니다.")
                mode = "LIDAR"
                continue

            # 1. 정차 중 (PARKING)
            if park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    if mission_idx < len(MISSION):
                        park_state = "SEARCH"
                        search_sub_state = "ROTATE"
                        search_start_t = time.time()
                        last_seen_x = cx_mid + 40 
                        was_in_bottom = was_in_left = was_in_right = False
                        print(f"다음 미션 [{MISSION[mission_idx]}] 탐색 회전 시작")
                    continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            # 2. 객체 추적 중 (TRACK)
            elif found:
                park_state = "TRACK"
                w_cam = -KP_ROT * err_x
                
                # 조향각 w_cam이 클 경우(로봇 중심과 색지 중심 차이가 클 때) 전진 속도를 줄여 정렬 정밀도 확보
                v_adj = APPROACH_V * max(0.4, (1.0 - abs(err_x) / (W/2)))
                v, w = v_adj, w_cam
                
                send_cmd(v, w)
                cv2.putText(frame, f"TRACKING: {target} | Err: {err_x}", (10, 25), 0, 0.6, draw, 1)

            # 3. 객체 놓침 또는 다음 객체 탐색 (SEARCH)
            else:
                # 정면 바닥 구역에 정상 도달했다가 시야에서 완전히 사라진 게 확실할 때만 골인 인정
                if was_in_bottom and not was_in_left and not was_in_right:
                    park_state = "PARKING"
                    park_t = time.time()
                    was_in_bottom = was_in_left = was_in_right = False
                    print(f"[{target}] 정면 골인 인식을 통한 확실한 도착 판정!")
                else:
                    # 색지를 놓쳤을 때 수행하는 탐색(SEARCH) 루프
                    park_state = "SEARCH"
                    now = time.time()
                    
                    if was_in_left:
                        v, w = 0.0, 1.6  
                        cv2.putText(frame, "SNAP TURN: LEFT ESCAPE", (10, 50), 0, 0.5, (0, 0, 255), 2)
                        # 탈출 회전 시에는 회전 타임아웃 타이머 리셋
                        search_start_t = now
                    elif was_in_right:
                        v, w = 0.0, -1.6 
                        cv2.putText(frame, "SNAP TURN: RIGHT ESCAPE", (10, 50), 0, 0.5, (0, 0, 255), 2)
                        search_start_t = now
                    else:
                        # 🔄 [핵심 수정] 2바퀴 회전 후 미발견 시 5cm 전진 패턴 제어
                        if search_sub_state == "ROTATE":
                            # 기본 제자리 회전 탐색
                            v = 0.0
                            w = -1.3 if last_seen_x > cx_mid else 1.3
                            
                            # 2바퀴 회전 시간(약 4.5초) 초과 시 전진 상태로 전환
                            if now - search_start_t >= ROTATION_2_TURNS_TIME:
                                print(" [탐색 실패] 2바퀴 회전 동안 발견 불가 -> 5cm 전진 시퀀스 작동")
                                search_sub_state = "GO_FORWARD"
                                search_start_t = now
                                
                        elif search_sub_state == "GO_FORWARD":
                            # 약 5cm 전진 제어 (0.25초 동안 속도 0.2)
                            v = 0.2
                            w = 0.0
                            
                            # 시간 다 되면 다시 회전 상태로 복귀
                            if now - search_start_t >= MOVE_5CM_TIME:
                                print(" 5cm 이동 완료 -> 해당 지점에서 다시 제자리 회전 탐색")
                                search_sub_state = "ROTATE"
                                search_start_t = now
                    
                    # 오작동 방지 플래그 리셋
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
