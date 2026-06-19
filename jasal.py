import cv2
import serial
import numpy as np
import time
import math
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

# ── CONTROL SMOOTHING (LPF) ──────────────────────────────────────────
prev_v, prev_w = 0.0, 0.0
ALPHA_CMD = 0.4  # 제어값 평활화 가중치

def send_cmd_smooth(v, w):
    global prev_v, prev_w
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    # LPF (Low Pass Filter) 적용
    v = (1 - ALPHA_CMD) * prev_v + ALPHA_CMD * v
    w = (1 - ALPHA_CMD) * prev_w + ALPHA_CMD * w
    prev_v, prev_w = v, w
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): 
    global prev_v, prev_w
    prev_v, prev_w = 0.0, 0.0
    arduino_ser.write(b"0.000,0.000\n")

# ── CAMERA & LIDAR INITIALIZATION ─────────────────────────────────────
# (생략: 기존 초기화 코드 사용)
cap = cv2.VideoCapture(0)
# ... (설정값 동일)

# ── LIDAR THREAD ──────────────────────────────────────────────────────
# (생략: 기존 Lidar Thread 로직 유지)

# ── PARAMS & STATE ────────────────────────────────────────────────────
WALL_CONFIRM = 5 
DETECT_CONFIRM = 6
wall_state_count = 0
detect_count = 0
mode, lidar_state, park_state = "LIDAR", "WALL_SEARCH", "TRACK"
mission_idx = 0
follow_side = "L"

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        loop_start = time.time()
        ret, frame = cap.read()
        if not ret: continue

        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan = get_scan()
        fm = front_min(scan)
        adir = avoid_dir(scan)

        # 미션 종료 처리
        if mission_idx >= len(MISSION):
            stop_robot()
            continue

        # 비전 처리
        target = MISSION[mission_idx]
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        # ══ LIDAR 모드 ════════════════════════════════════════════════
        if mode == "LIDAR":
            detect_count = detect_count + 1 if found else 0
            if detect_count >= DETECT_CONFIRM:
                mode, park_state = "PARK", "TRACK"
                continue

            if lidar_state == "WALL_SEARCH":
                if fm < WALL_SCAN_DIST:
                    lidar_state = "WALL_APPROACH"
                else:
                    send_cmd_smooth(0.0, WALL_SEARCH_W)
            
            elif lidar_state == "WALL_APPROACH":
                sd = side_dist(scan, follow_side)
                if sd <= WALL_TARGET * 1.3:
                    wall_state_count += 1
                    if wall_state_count >= WALL_CONFIRM:
                        lidar_state = "WALL_FOLLOW"
                        wall_state_count = 0
                else:
                    wall_state_count = 0
                    v, w = WALL_APPROACH_V, (1 if follow_side == "L" else -1) * 0.3
                    send_cmd_smooth(v, w)
            
            elif lidar_state == "WALL_FOLLOW":
                v, w = wall_follow(scan, fm, adir, follow_side)
                send_cmd_smooth(v, w)

        # ══ PARK 모드 ════════════════════════════════════════════════
        elif mode == "PARK":
            # (각 상태머신 내 기존 send_cmd를 모두 send_cmd_smooth로 교체)
            # 예: TRACK 상태
            if park_state == "TRACK" and found:
                # ... 기존 로직 수행 ...
                send_cmd_smooth(v, w)
            # ... 나머지 상태 동일 적용 ...

        # ── 루프 주기 고정 (50Hz = 0.02초) ──────────────────────────
        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break
        
        elapsed = time.time() - loop_start
        time.sleep(max(0, 0.02 - elapsed))

except KeyboardInterrupt:
    print("STOP")
finally:
    stop_robot()
    cap.release()
    cv2.destroyAllWindows()
