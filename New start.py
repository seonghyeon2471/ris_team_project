import cv2
import serial
import numpy as np
import time
import math
import threading

arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR OK")

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

def get_scan():
    with scan_lock:
        return _scan_pub.copy()

def front_min(scan):
    idx = np.arange(-FRONT_RANGE, FRONT_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -2.8, 2.8)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

COLOR_CFG = {
    "red": {"hsv1": ([169, 168, 96], [179, 222, 157]), "hsv2": None, "bgr": ([20, 20, 80], [255, 255, 255]), "draw": (0, 0, 255)},
    "yellow": {"hsv1": ([16, 137, 142], [30, 214, 195]), "hsv2": None, "bgr": ([0, 80, 80], [255, 255, 255]), "draw": (0, 200, 255)},
    "blue": {"hsv1": ([106, 168, 54], [131, 210, 82]), "hsv2": None, "bgr": ([40, 0, 0], [255, 220, 220]), "draw": (255, 80, 0)},
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

MIN_AREA = 400
KP_ROT = 0.003
APPROACH_V = 0.22
PARK_SEC = 1.2
DETECT_CONFIRM = 6
PARK_START_DIST = 30.0
BOTTOM_10PCT = int(240 * 0.85)

mode = "LIDAR"
mission_idx = 0
detect_count = 0
park_state = "TRACK"
last_seen_x = 160
last_bottom_y = 0
was_in_bottom = False
park_t = None

# [핵심 수정] 라이다 회피 상태 관리
avoiding = False          # 회피 중 플래그
avoid_start_dist = None   # 회피 시작 거리
returning = False         # 회피 후 복귀 중 플래그

print(f"START | MISSION: {MISSION}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]
        cx_mid = W // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan = get_scan()
        fm = front_min(scan)
        adir = avoid_dir(scan)
        err_x = 0

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw = COLOR_CFG[target]["draw"]

        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)

        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox = bx + bw // 2
            by_bot = min(by_top + bh, 239)
            err_x = ox - cx_mid
            last_seen_x = ox
            last_bottom_y = by_bot
            was_in_bottom = (by_bot >= BOTTOM_10PCT)

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)

        # ══ LIDAR 모드: 라이다 우선, 카메라 추적 일시정지 ═════════════
        if mode == "LIDAR":
            if found:
                detect_count += 1
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                avoiding = False
                returning = False
                print(f"[{target}] 발견 → 추적 시작")
                continue

            # [핵심 수정] 라이다 장애물 회피 우선 (카메라 추적 중단)
            if fm < THRESH_STOP:
                v, w = 0.08, adir * 1.2  # 회피 속도 느리게, 조향 강하게
                avoiding = True
                cv2.putText(frame, "AVOIDING (camera OFF)", (10, 50), 0, 0.5, (255, 0, 0), 2)
            elif fm < THRESH_TURN:
                v, w = 0.12, adir * 1.0
                avoiding = True
                cv2.putText(frame, "AVOIDING...", (10, 50), 0, 0.5, (255, 0, 0), 2)
            elif fm < THRESH_SLOW:
                v, w = 0.18, adir * 0.5
            else:
                v, w = 0.28, 0.0
                if avoiding:
                    avoiding = False  # 회피 완료
                    print("[회피 완료] 카메라 추적 재개")

            send_cmd(v, w)
            cv2.putText(frame, "MODE: LIDAR", (10, 25), 0, 0.5, (255, 255, 255), 1)

        # ══ PARK 모드: 카메라 추적 + 라이다 보조 ════════════════
        elif mode == "PARK":
            if park_state == "PARKING":
                stop_robot()
                elapsed = time.time() - park_t if park_t else 0
                if elapsed >= PARK_SEC:
                    mission_idx += 1
                    print(f"[PARKING 완료] {mission_idx-1} → {mission_idx}")

                    if mission_idx < len(MISSION):
                        park_state = "SEARCH"
                        last_seen_x = cx_mid + 40
                        was_in_bottom = False
                        avoiding = False
                        returning = False
                        print(f"다음 미션 [{MISSION[mission_idx]}] 탐색")
                        continue
                    else:
                        stop_robot()
                        cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.imshow("f", frame)
                        cv2.waitKey(1)
                        continue
                cv2.putText(frame, f"PARKING: {target}", (10, 25), 0, 0.6, draw, 2)

            elif found and not avoiding:
                # [핵심 수정] 회피 중일 때만 카메라 추적 중단
                park_state = "TRACK"

                if fm < PARK_START_DIST:
                    if abs(err_x) < 20:
                        park_state = "PARKING"
                        park_t = time.time()
                        avoiding = False
                        returning = False
                        print(f"[{target}] 주차 시작 (dist={fm:.1f}cm)")
                        continue

                if fm >= THRESH_SLOW:
                    v, w = APPROACH_V, -KP_ROT * err_x
                else:
                    w_cam, w_lid = -KP_ROT * err_x, adir * 0.5
                    if fm < THRESH_STOP:
                        v, w = 0.09, w_lid
                    elif fm < THRESH_TURN:
                        v, w = 0.13, 0.6 * w_lid + 0.4 * w_cam
                    else:
                        v, w = 0.18, 0.4 * w_lid + 0.6 * w_cam
                send_cmd(v, w)
                cv2.putText(frame, f"TRACKING: {target}", (10, 25), 0, 0.6, draw, 1)

            else:
                # 회피 중이거나 객체 놓침
                if avoiding:
                    # 회피 계속 (카메라 추적 안 함)
                    if fm >= THRESH_SLOW:
                        avoiding = False
                        print("[회피 완료] 추적 재개")
                    v, w = 0.10, adir * 1.0
                    cv2.putText(frame, "AVOIDING (tracking OFF)", (10, 50), 0, 0.5, (255, 0, 0), 2)
                else:
                    if park_state != "SEARCH":
                        park_state = "SEARCH"

                    if was_in_bottom:
                        park_state = "PARKING"
                        park_t = time.time()
                        was_in_bottom = False
                        print(f"[{target}] 주차")
                    else:
                        v = 0.0
                        w = -1.3 if last_seen_x > cx_mid else 1.3
                        send_cmd(v, w)
                        cv2.putText(frame, f"SEARCHING: {target}", (10, 25), 0, 0.6, (0, 255, 255), 1)

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
