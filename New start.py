import cv2
import serial
import numpy as np
import time
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# ── CAMERA ────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR FILTER ──────────────────────────────────────────────────────
EMA_ALPHA   = 0.35
MEDIAN_K    = 2
FRONT_RANGE = 45

THRESH_STOP = 18.0
THRESH_TURN = 35.0
THRESH_SLOW = 55.0
OBS_LIMIT   = 100.0

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
        if 3 < dist_cm < 1200:
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
        "hsv": [(np.array([95, 156, 24]), np.array([115, 223, 66]))],
        "draw": (255, 80, 0),
    },
}
MISSION = ["red", "yellow", "blue"]

def make_mask(hsv_img, name):
    cfg = COLOR_CFG[name]
    mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
    for lo, hi in cfg["hsv"]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv_img, lo, hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def centroid_of_contour(cnt):
    m = cv2.moments(cnt)
    if m["m00"] == 0:
        return None, None
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])

# ── PARAMS ────────────────────────────────────────────────────────────
MIN_AREA       = 400
KP_X           = 0.003
KP_Y           = 0.004
W_MAX          = 0.85
V_MAX          = 0.20
V_MIN          = 0.06

TARGET_Y_RATIO  = 0.72
CX_OK_PX        = 12
CY_OK_PX        = 12

DETECT_CONFIRM  = 6
ARRIVE_CONFIRM  = 8
ALIGN_FWD_SEC   = 0.22
ALIGN_FWD_V     = 0.08
HOLD_SEC        = 1.0
PARK_SEC        = 1.2

PARK_ENTRY_GRACE = 8

# LIDAR 우선순위
OBS_EMERGENCY = 12.0
OBS_STOP      = 18.0
OBS_TURN      = 35.0
OBS_SLOW      = 55.0

# ── STATE ─────────────────────────────────────────────────────────────
mission_idx      = 0
mode             = "TRACK"   # TRACK / ALIGN_FWD / HOLD / PARKING / SEARCH
detect_count     = 0
arrive_count     = 0
align_t          = None
hold_t           = None
park_t           = None
park_entry_count = 0
last_seen_x      = 160

print("START")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]
        cx_mid = W // 2
        cy_mid = int(H * TARGET_Y_RATIO)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan = get_scan()
        fm = front_min(scan)
        adir = avoid_dir(scan)

        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL DONE", (60, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            cv2.imshow("f", frame)
            cv2.waitKey(1)
            continue

        target = MISSION[mission_idx]
        draw = COLOR_CFG[target]["draw"]

        mask = make_mask(hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        area = 0
        centroid_x = None
        centroid_y = None
        err_x = 0
        err_y = 0

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

            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), draw, 2)
            cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 255, 255), -1)
            cv2.circle(frame, (cx_mid, cy_mid), 5, (255, 255, 255), -1)
            cv2.line(frame, (centroid_x, centroid_y), (cx_mid, cy_mid), (100, 100, 255), 1)
            cv2.putText(frame, f"ex={err_x:+d} ey={err_y:+d} area={int(area)}",
                        (bx, max(by - 8, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, draw, 1)

        cv2.line(frame, (cx_mid, 0), (cx_mid, H), (120, 120, 120), 1)
        cv2.line(frame, (0, cy_mid), (W, cy_mid), (120, 120, 120), 1)

        # ── LIDAR 우선: 위험하면 카메라보다 먼저 회피 ───────────────
        if fm < OBS_EMERGENCY:
            stop_robot()
            cv2.putText(frame, f"EMERGENCY STOP front={fm:.0f}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        if fm < OBS_STOP:
            v, w = 0.0, adir * 1.0
            label = "OBS_STOP"
            send_cmd(v, w)
            cv2.putText(frame, f"{label} front={fm:.0f}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        if fm < OBS_TURN:
            v_ob = 0.08
            w_ob = adir * 0.9
            label = "OBS_TURN"
            if mode == "TRACK" and found:
                w_cam = -KP_X * err_x
                w = 0.7 * w_ob + 0.3 * w_cam
                v = v_ob
            else:
                v, w = v_ob, w_ob
            send_cmd(v, w)
            cv2.putText(frame, f"{label} front={fm:.0f}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        if fm < OBS_SLOW:
            if mode == "TRACK" and found:
                w = -KP_X * err_x
                raw_v = KP_Y * err_y
                v = float(np.clip(raw_v, V_MIN, V_MAX)) if raw_v > 0 else 0.0
            else:
                v = 0.14
                w = 0.0
            send_cmd(v, w)
            cv2.putText(frame, f"OBS_SLOW front={fm:.0f}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        # ── 일반 TRACK ───────────────────────────────────────────────
        if mode == "TRACK":
            if found:
                detect_count += 1
                cv2.putText(frame, f"DETECT [{detect_count}/{DETECT_CONFIRM}]",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 2)
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                detect_count = 0
                arrive_count = 0
                mode = "APPROACH"
                print(f"[{target}] 인식 확정 -> APPROACH")
                cv2.imshow("f", frame)
                cv2.waitKey(1)
                continue

            send_cmd(0.22, 0.0)
            cv2.putText(frame, f"TRACK [{target}] front={fm:.0f}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)

        # ── APPROACH: 센트로이드 기준 정렬 ─────────────────────────
        elif mode == "APPROACH":
            if not found:
                mode = "SEARCH"
                print(f"[{target}] 소실 -> SEARCH")
            else:
                x_ok = abs(err_x) <= CX_OK_PX
                y_ok = abs(err_y) <= CY_OK_PX

                if x_ok and y_ok and area >= 1200:
                    arrive_count += 1
                    cv2.putText(frame, f"ARRIVE [{arrive_count}/{ARRIVE_CONFIRM}]",
                                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    if abs(err_x) > CX_OK_PX * 2 or abs(err_y) > CY_OK_PX * 2:
                        arrive_count = 0

                if arrive_count >= ARRIVE_CONFIRM:
                    mode = "ALIGN_FWD"
                    align_t = time.time()
                    arrive_count = 0
                    print(f"[{target}] 중심 일치 -> final forward")
                else:
                    w_out = float(np.clip(-KP_X * err_x, -W_MAX, W_MAX))
                    raw_v = KP_Y * err_y
                    v_out = float(np.clip(raw_v, V_MIN, V_MAX)) if raw_v > 0 else 0.0
                    send_cmd(v_out, w_out)
                    cv2.putText(frame,
                                f"APPROACH [{target}] ex={err_x:+d} ey={err_y:+d} v={v_out:.2f} w={w_out:.2f}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw, 1)

        # ── ALIGN_FWD: 색 위로 조금 더 진입 ─────────────────────────
        elif mode == "ALIGN_FWD":
            if not found:
                stop_robot()
                mode = "SEARCH"
                print(f"[{target}] ALIGN_FWD 중 소실 -> SEARCH")
            else:
                elapsed = time.time() - align_t
                if elapsed < ALIGN_FWD_SEC:
                    send_cmd(ALIGN_FWD_V, 0.0)
                    cv2.putText(frame, f"ALIGN_FWD {elapsed:.2f}s",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, draw, 2)
                else:
                    stop_robot()
                    mode = "HOLD"
                    hold_t = time.time()
                    print(f"[{target}] 색 위 안착 -> HOLD")

        # ── HOLD: 색 위에서 1초 정지 ─────────────────────────────────
        elif mode == "HOLD":
            stop_robot()
            elapsed = time.time() - hold_t
            remain = max(0.0, HOLD_SEC - elapsed)
            cv2.putText(frame, f"HOLD [{target}] {remain:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw, 2)

            if elapsed >= HOLD_SEC:
                mode = "PARKING"
                park_t = time.time()
                print(f"[{target}] HOLD 완료 -> PARKING")

        # ── PARKING: 다음 색으로 이동 ──────────────────────────────
        elif mode == "PARKING":
            stop_robot()
            elapsed = time.time() - park_t
            remain = max(0.0, PARK_SEC - elapsed)
            cv2.putText(frame, f"PARKING [{target}] {remain:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw, 2)

            if elapsed >= PARK_SEC:
                mission_idx += 1
                mode = "TRACK"
                detect_count = 0
                arrive_count = 0
                park_entry_count = 0
                print(f"[{target}] 완료 -> 다음 색")

        # ── SEARCH ──────────────────────────────────────────────────
        elif mode == "SEARCH":
            if found:
                mode = "APPROACH"
                print(f"[{target}] 재검출 -> APPROACH")
            else:
                stop_robot()
                w = -1.30 if last_seen_x > cx_mid else 1.30
                send_cmd(0.0, w)
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
