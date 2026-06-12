import cv2
import serial
import numpy as np
import time
import threading

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# =========================================
# LIDAR BOOT
# =========================================
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR OK")

# =========================================
# LIDAR PARAMETERS (첫 번째 코드 기반)
# =========================================
MAX_SPEED = 0.30
MIN_SPEED = 0.09
MAX_W = 0.9
THRESH_30 = 32.0
THRESH_20 = 22.0
THRESH_10 = 12.0
FRONT_CHECK_RANGE = 45

EMA_ALPHA = 0.35
MEDIAN_K = 2

scan_data = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

# =========================================
# LIDAR UTILS (첫 번째 코드 기반)
# =========================================
def apply_ema(angle, new_dist_cm):
    if not isinstance(new_dist_cm, (int, float)) or new_dist_cm <= 0:
        return
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():
    k = MEDIAN_K
    window = 2 * k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        values = np.sort(scan_data[idx])
        filtered[i] = values[window // 2]
    scan_data[:] = filtered

def get_front_min():
    idx = np.arange(-FRONT_CHECK_RANGE, FRONT_CHECK_RANGE + 1) % 360
    return float(np.min(scan_data[idx]))

def choose_avoid_direction():
    left_avg = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    return 1 if left_avg >= right_avg else -1

# =========================================
# LIDAR THREAD
# =========================================
def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue
        s_flag = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - s_flag) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150:
            apply_ema(angle, dist_cm)
        if s_flag != 1:
            continue
        apply_median_filter()

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock:
        return scan_data.copy()

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# COLOR CONFIG (두 번째 코드 기반)
# =========================================
COLOR_CFG = {
    "red": {
        "hsv1": ([169, 168, 96], [179, 222, 157]),
        "hsv2": None,
        "bgr": ([20, 20, 80], [255, 255, 255]),
        "draw": (0, 0, 255)
    },
    "yellow": {
        "hsv1": ([16, 137, 142], [30, 214, 195]),
        "hsv2": None,
        "bgr": ([0, 80, 80], [255, 255, 255]),
        "draw": (0, 200, 255)
    },
    "blue": {
        "hsv1": ([106, 168, 54], [131, 210, 82]),
        "hsv2": None,
        "bgr": ([40, 0, 0], [255, 220, 220]),
        "draw": (255, 80, 0)
    },
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

# =========================================
# PARK PARAMETERS (두 번째 코드 기반)
# =========================================
MIN_AREA = 400
KP_ROT = 0.003
APPROACH_V = 0.22
PARK_SEC = 1.2
BOTTOM_10PCT = int(240 * 0.94)  # 225px
PARK_X_TOLERANCE = 20  # 중앙 20px 이내

# =========================================
# STATE
# =========================================
mission_idx = 0
parking = False
park_t = None
last_seen_x = 160

print(f"START | MISSION: {MISSION}")
print("라이다 회피 우선 + 가다가 색상 발견 시 즉시 주차")

# =========================================
# MAIN LOOP
# =========================================
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]
        cx_mid = W // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw = COLOR_CFG[target]["draw"]

        # =========================================
        # COLOR DETECTION
        # =========================================
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)

        err_x = 0
        by_bot = 0

        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox = bx + bw // 2
            by_bot = min(by_top + bh, 239)
            err_x = ox - cx_mid
            last_seen_x = ox

            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)

        # =========================================
        # PARKING STATES
        # =========================================
        if parking:
            stop_robot()
            elapsed = time.time() - park_t
            cv2.putText(frame, f"PARKING: {target} ({elapsed:.1f}s)", (10, 25), 0, 0.6, draw, 2)

            if elapsed >= PARK_SEC:
                mission_idx += 1
                parking = False
                park_t = None
                if mission_idx < len(MISSION):
                    print(f"다음 미션 [{MISSION[mission_idx]}] 회피 주행 시작")
                else:
                    stop_robot()
                    cv2.putText(frame, "ALL MISSIONS DONE", (30, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.imshow("f", frame)
                    cv2.waitKey(1)
                    continue
            continue

        # =========================================
        # 라이다 회피 우선 + 색상 발견 시 즉시 주차
        # =========================================
        scan = get_scan()
        front_min = get_front_min()
        avoid_dir = choose_avoid_direction()

        # [핵심] 색상 발견 시 즉시 주차 (뱅글뱅글 안 도는게)
        if found:
            if by_bot >= BOTTOM_10PCT and abs(err_x) < PARK_X_TOLERANCE:
                # 가까우면서 중앙이면 즉시 주차
                parking = True
                park_t = time.time()
                cv2.putText(frame, f"PARKING NOW: {target}", (10, 50), 0, 0.7, draw, 2)
                print(f"[{target}] 즉시 주차 (by_bot={by_bot}, err_x={err_x})")
                continue
            else:
                # 색상이 가까워지면 추적, 아니면 회피 계속
                if front_min >= THRESH_30:
                    v = APPROACH_V
                    w = -KP_ROT * err_x
                else:
                    w_cam = -KP_ROT * err_x
                    w_lid = avoid_dir * 0.5
                    if front_min < THRESH_10:
                        v, w = 0.09, w_lid
                    elif front_min < THRESH_20:
                        v, w = 0.12, 0.6 * w_lid + 0.4 * w_cam
                    else:
                        v, w = 0.15, 0.4 * w_lid + 0.6 * w_cam
                send_cmd(v, w)
                cv2.putText(frame, f"TRACKING → PARK: {target}", (10, 25), 0, 0.5, (0, 255, 0), 2)
        else:
            # [핵심] 색상이 안 보이면 라이다 회피 주행 계속 (첫 번째 코드 기반)
            if front_min < THRESH_10:
                v = MIN_SPEED
                w = avoid_dir * MAX_W
                cv2.putText(frame, f"VERY CLOSE: {front_min:.1f}cm", (10, 25), 0, 0.5, (255, 0, 0), 2)
            elif front_min < THRESH_20:
                v = 0.12
                w = avoid_dir * 0.8
                cv2.putText(frame, f"CRITICAL: {front_min:.1f}cm", (10, 25), 0, 0.5, (255, 0, 0), 2)
            elif front_min < THRESH_30:
                v = 0.15
                w = avoid_dir * 0.7
                cv2.putText(frame, f"WARNING: {front_min:.1f}cm", (10, 25), 0, 0.5, (255, 165, 0), 2)
            else:
                v = MAX_SPEED
                w = 0.0
                cv2.putText(frame, "LIDAR: FORWARD", (10, 25), 0, 0.5, (255, 255, 255), 1)

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
    lidar_ser.close()
    cv2.destroyAllWindows()
