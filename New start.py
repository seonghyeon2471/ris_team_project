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
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 45
THRESH_SLOW = 55.0
THRESH_TURN = 35.0
THRESH_STOP = 18.0

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

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# ── COLOR CONFIG ──────────────────────────────────────────────────────
COLOR_CFG = {
    "red": {
        "hsv": [(np.array([169, 168, 96]), np.array([179, 222, 157]))],
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv": [(np.array([16, 137, 142]), np.array([30, 214, 195]))],
        "draw": (0, 200, 255),
    },
    "blue": {
        "hsv": [(np.array([106, 168, 54]), np.array([131, 210, 82]))],
        "draw": (255, 80, 0),
    },
}
MISSION = ["red", "yellow", "blue"]

def make_mask(frame, hsv_img, name):
    cfg = COLOR_CFG[name]
    m = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
    for lo, hi in cfg["hsv"]:
        m = cv2.bitwise_or(m, cv2.inRange(hsv_img, lo, hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
    return m

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_ROT         = 0.003
KP_FWD         = 0.0025
APPROACH_V     = 0.22
PARK_SEC       = 1.2
DETECT_CONFIRM = 6
BOTTOM_10PCT   = int(240 * 0.90)

CX_OK_PX       = 18
CY_OK_PX       = 18
ON_COLOR_AREA  = 1200
ALIGN_FWD_SEC  = 0.25
ALIGN_FWD_V    = 0.08
STAY_ON_SEC    = 1.0
SEARCH_SPIN_W  = 1.30

# ── STATE ─────────────────────────────────────────────────────────────
mode           = "LIDAR"
mission_idx    = 0
detect_count   = 0

park_state     = "TRACK"
last_seen_x    = 160
last_seen_y    = 0
last_bottom_y  = 0
was_in_bottom  = False
park_t         = None
forward_t      = None
on_t           = None

PARK_ENTRY_GRACE = 8
park_entry_count = 0

print(f"START | 하단 10% 기준: y > {BOTTOM_10PCT}px")

def centroid_of_contour(cnt):
    m = cv2.moments(cnt)
    if m["m00"] == 0:
        return None, None
    cx = int(m["m10"] / m["m00"])
    cy = int(m["m01"] / m["m00"])
    return cx, cy

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = W // 2
        cy_mid = int(H * 0.72)
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL DONE", (60, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        centroid_x = centroid_y = None
        area = 0
        if found:
            area = cv2.contourArea(big)
            bx, by, bw, bh = cv2.boundingRect(big)
            centroid_x, centroid_y = centroid_of_contour(big)

            if centroid_x is None:
                centroid_x = bx + bw // 2
                centroid_y = by + bh // 2

            err_x = centroid_x - cx_mid
            err_y = centroid_y - cy_mid

            last_seen_x = centroid_x
            last_seen_y = centroid_y
            last_bottom_y = min(by + bh, H - 1)
            was_in_bottom = (last_bottom_y >= BOTTOM_10PCT)

            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), draw, 2)
            cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 255, 255), -1)
            cv2.circle(frame, (cx_mid, cy_mid), 5, (255, 255, 255), -1)
            cv2.line(frame, (centroid_x, centroid_y), (cx_mid, cy_mid), (100, 100, 255), 1)
            cv2.putText(frame, f"ex={err_x:+d} ey={err_y:+d} area={int(area)}",
                        (bx, max(by - 8, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, draw, 1)

        cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)

        if mode == "LIDAR":
            if found:
                detect_count += 1
                cv2.putText(frame, f"DETECT [{detect_count}/{DETECT_CONFIRM}]",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 2)
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                mode = "PARK"
                park_state = "TRACK"
                last_bottom_y = 0
                was_in_bottom = False
                park_t = None
                park_entry_count = 0
                stop_robot()
                print(f"[{target}] 인식 확정 -> PARK")
                cv2.imshow("f", frame)
                cv2.waitKey(1)
                continue

            if fm < THRESH_STOP:
                v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN:
                v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW:
                v, w = 0.18, adir * 0.4
            else:
                v, w = 0.28, 0.0

            send_cmd(v, w)
            cv2.putText(frame, f"LIDAR front={fm:.0f}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        if park_state == "PARKING":
            stop_robot()
            elapsed = time.time() - park_t
            remain = max(0.0, PARK_SEC - elapsed)
            cv2.putText(frame, f"PARKING [{target}] {remain:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw, 2)
            cv2.imshow("f", frame)
            if elapsed >= PARK_SEC:
                mission_idx += 1
                mode = "LIDAR"
                park_state = "TRACK"
                last_bottom_y = 0
                was_in_bottom = False
                detect_count = 0
                park_entry_count = 0
                print(f"[{target}] 완료 -> LIDAR 복귀")
            cv2.waitKey(1)
            continue

        if park_entry_count < PARK_ENTRY_GRACE:
            park_entry_count += 1
            send_cmd(APPROACH_V, 0.0)
            cv2.putText(frame, f"PARK ENTRY stabilize ({park_entry_count}/{PARK_ENTRY_GRACE})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 1)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        if found:
            park_state = "TRACK"

            x_ok = abs(err_x) <= CX_OK_PX
            y_ok = abs(err_y) <= CY_OK_PX

            if x_ok and y_ok and area >= ON_COLOR_AREA:
                park_state = "PARKING"
                park_t = time.time()
                stop_robot()
                print(f"[{target}] centroid align + area OK -> PARKING")
                cv2.imshow("f", frame)
                cv2.waitKey(1)
                continue

            if fm >= THRESH_SLOW:
                v = APPROACH_V
                w = -KP_ROT * err_x
                label = "LIDAR_OFF"
            else:
                w_cam = -KP_ROT * err_x
                w_lidar = adir * 0.7
                if fm < THRESH_STOP:
                    v, w = 0.09, w_lidar
                elif fm < THRESH_TURN:
                    v = 0.13
                    w = 0.7 * w_lidar + 0.3 * w_cam
                else:
                    v = 0.18
                    w = 0.3 * w_lidar + 0.7 * w_cam
                label = "LIDAR_ON"

            if not (x_ok and y_ok):
                v = 0.18 if area < ON_COLOR_AREA else APPROACH_V
                w = -KP_ROT * err_x

            send_cmd(v, w)
            cv2.putText(frame, f"TRACK [{target}] {label} fm={fm:.0f} area={int(area)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.48, draw, 1)

        else:
            if was_in_bottom:
                park_state = "PARKING"
                park_t = time.time()
                stop_robot()
                print(f"[{target}] 하단 소실 (last_y={last_bottom_y}px) -> {PARK_SEC}s 정차")
            else:
                park_state = "SEARCH"
                v = 0.0
                w = -1.30 if last_seen_x > cx_mid else 1.30
                send_cmd(v, w)
                cv2.putText(frame, f"SEARCH [{target}] last_x={last_seen_x}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("INTERRUPTED")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    cv2.destroyAllWindows()
    print("SHUTDOWN")
