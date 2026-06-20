import cv2
import serial
import numpy as np
import time
import math
import threading

# ── [1] 초기화 ──────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)

# ── [2] LIDAR 쓰레드 ────────────────────────────────────────────────────
_scan = np.full(360, 150.0, dtype=np.float32)
_scan_pub = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3: continue
        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist < 150: _scan[angle] = (1 - 0.35) * _scan[angle] + 0.35 * dist
        if sf == 1:
            with scan_lock: _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

# ── [3] 핵심 함수 ──────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4); w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)

def wall_follow(scan, fm, adir, side):
    sign = 1 if side == "L" else -1
    if fm < 18.0: return (0.08, adir * 1.1)
    if fm < 30.0: return (0.10, adir * 0.85)
    sd = np.mean(scan[np.arange(85, 96) % 360 if side == "L" else np.arange(265, 276) % 360])
    err = sd - 20.0
    w = np.clip(sign * 0.012 * err, -0.9, 0.9)
    v = 0.22 if fm > 55.0 else 0.15
    return (v, w)

def side_dist_avg(scan, side):
    return np.mean(scan[np.arange(85, 96) % 360 if side == "L" else np.arange(265, 276) % 360])

# ── [4] 설정 및 변수 ────────────────────────────────────────────────────
MISSION = ["red", "yellow", "blue"]
COLOR_CFG = {
    "red": {"hsv1": ([169, 136, 114], [179, 220, 255]), "bgr": ([20, 20, 80], [255, 255, 255])},
    "yellow": {"hsv1": ([24, 19, 193], [45, 165, 255]), "bgr": ([0, 80, 80], [255, 255, 255])},
    "blue": {"hsv1": ([98, 100, 95], [138, 207, 246]), "bgr": ([40, 0, 0], [255, 220, 220])}
}

mission_idx, mode, park_state = 0, "LIDAR", "TRACK"
lidar_state, arrive_count, detect_count, follow_side = "WALL_SEARCH", 0, 0, "L"
t_start = 0

# 주차 직후 상태를 판별하는 플래그 추가
just_parked = False

# ── [5] 메인 루프 ──────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue
        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan = _scan_pub.copy()
        fm = np.min(scan[np.arange(-90, 91) % 360])
        adir = 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1
        
        target = MISSION[mission_idx] if mission_idx < len(MISSION) else None
        found = False
        if target:
            cfg = COLOR_CFG[target]
            mask = cv2.bitwise_and(cv2.inRange(hsv, np.array(cfg["hsv1"][0]), np.array(cfg["hsv1"][1])),
                                   cv2.inRange(frame, np.array(cfg["bgr"][0]), np.array(cfg["bgr"][1])))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                big = max(cnts, key=cv2.contourArea)
                if cv2.contourArea(big) > 400:
                    found = True
                    M = cv2.moments(big)
                    cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])

        if mission_idx >= len(MISSION): stop_robot(); break

        if mode == "LIDAR":
            if found: detect_count += 1
            else: detect_count = 0
            if detect_count >= 6: mode = "PARK"; park_state = "TRACK"
            else:
                if lidar_state == "WALL_SEARCH":
                    if fm < 150:
                        # 주차 직후면 가까운 쪽(<=), 평상시면 좀 떨어져 있는 넓은 쪽(>)으로 이동
                        if just_parked:
                            follow_side = "L" if np.mean(scan[85:96]) <= np.mean(scan[265:276]) else "R"
                            just_parked = False
                        else:
                            follow_side = "L" if np.mean(scan[85:96]) > np.mean(scan[265:276]) else "R"
                        lidar_state = "WALL_APPROACH"
                    else: send_cmd(0, 1.1)
                elif lidar_state == "WALL_APPROACH":
                    if np.mean(scan[85:96] if follow_side == "L" else scan[265:276]) < 26: lidar_state = "WALL_FOLLOW"
                    else: send_cmd(0.2, (1 if follow_side == "L" else -1) * 0.3)
                elif lidar_state == "WALL_FOLLOW":
                    v, w = wall_follow(scan, fm, adir, follow_side)
                    send_cmd(v, w)

        elif mode == "PARK":
            if park_state == "TRACK":
                if found:
                    err = cx - 160
                    if abs(err) < 30 and cy > 200: arrive_count += 1
                    else: arrive_count = 0
                    if arrive_count >= 8: park_state = "FORWARD"; t_start = time.time()
                    else:
                        w_cam = np.clip(-0.03 * err, -0.9, 0.9)
                        v = 0.10 if fm < 55 else 0.17
                        send_cmd(v, w_cam * 0.4 + adir * 0.6)
                else: send_cmd(0.0, 1.0)
            elif park_state == "FORWARD":
                send_cmd(0.12, 0.0)
                if time.time() - t_start > 0.4: park_state = "PARKING"; t_stop = time.time()
            elif park_state == "PARKING":
                stop_robot()
                if time.time() - t_stop > 2.0:
                    if MISSION[mission_idx] == "yellow": park_state = "SCAN_360"; t_start = time.time()
                    else: 
                        mission_idx += 1; mode = "LIDAR"; lidar_state = "WALL_SEARCH"
                        just_parked = True # 주차 완료 후 LIDAR 모드 복귀 시 플래그 ON
            elif park_state == "SCAN_360":
                send_cmd(0.0, 0.5)
                if time.time() - t_start > 3.0:
                    best_idx = np.argmin(scan)
                    follow_side = "L" if best_idx < 180 else "R"
                    mission_idx += 1; mode = "LIDAR"; lidar_state = "WALL_APPROACH"
            elif park_state in ["WALL_SEARCH", "WALL_APPROACH", "WALL_FOLLOW"]:
                if found: detect_count += 1
                else: detect_count = 0
                if detect_count >= 6: park_state = "TRACK"
                else:
                    if park_state == "WALL_SEARCH":
                        if fm < 150: 
                            # 여기서도 동일하게 just_parked 로직 적용
                            if just_parked:
                                follow_side = "L" if np.mean(scan[85:96]) <= np.mean(scan[265:276]) else "R"
                                just_parked = False
                            else:
                                follow_side = "L" if np.mean(scan[85:96]) > np.mean(scan[265:276]) else "R"
                            park_state = "WALL_APPROACH"
                        else: send_cmd(0, 1.1)
                    elif park_state == "WALL_APPROACH":
                        if side_dist_avg(scan, follow_side) < 26: park_state = "WALL_FOLLOW"
                        else: send_cmd(0.2, (1 if follow_side == "L" else -1) * 0.3)
                    elif park_state == "WALL_FOLLOW":
                        v, w = wall_follow(scan, fm, adir, follow_side)
                        send_cmd(v, w)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break
except: pass
finally: stop_robot(); cap.release(); cv2.destroyAllWindows()
