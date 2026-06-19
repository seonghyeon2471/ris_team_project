import cv2
import serial
import numpy as np
import time
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

# ── CAMERA ────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# ── LIDAR & CONTROL INITIALIZATION ────────────────────────────────────
# [이곳에 기존 Lidar Thread, get_scan, front_min, avoid_dir, wall_follow 등 함수 정의들을 삽입하세요]

prev_v, prev_w = 0.0, 0.0
ALPHA_CMD = 0.4 

def send_cmd_smooth(v, w):
    global prev_v, prev_w
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    v = (1 - ALPHA_CMD) * prev_v + ALPHA_CMD * v
    w = (1 - ALPHA_CMD) * prev_w + ALPHA_CMD * w
    prev_v, prev_w = v, w
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): 
    global prev_v, prev_w
    prev_v, prev_w = 0.0, 0.0
    arduino_ser.write(b"0.000,0.000\n")

# ── PARAMS & STATE ────────────────────────────────────────────────────
MISSION = ["red", "yellow", "blue"]
WALL_CONFIRM, DETECT_CONFIRM = 5, 6
wall_state_count, detect_count = 0, 0
mode, lidar_state, park_state = "LIDAR", "WALL_SEARCH", "TRACK"
mission_idx, follow_side = 0, "L"

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        loop_start = time.time()
        ret, frame = cap.read()
        if not ret: continue

        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # [수정 완료] get_scan() 함수 호출
        scan = get_scan() 
        fm = front_min(scan)
        adir = avoid_dir(scan)

        if mission_idx >= len(MISSION):
            stop_robot()
            continue

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
            if park_state == "TRACK" and found:
                # ... (로직 수행) ...
                send_cmd_smooth(v, w)

        # ── 루프 주기 고정 (50Hz) ────────────────────────────────────
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
