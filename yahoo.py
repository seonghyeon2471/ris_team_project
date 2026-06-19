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
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# ── LIDAR INITIALIZATION ──────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR DATA STRUCTURE ──────────────────────────────────────────────
_scan     = np.full(360, 150.0, dtype=np.float32)
_scan_pub = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

# ── FUNCTIONS ─────────────────────────────────────────────────────────
def _ema(a, d):
    if d > 0: _scan[a] = (1 - 0.35) * _scan[a] + 0.35 * d

def _median():
    k = 2
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
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3: continue
        angle = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150: _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock: _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock: return _scan_pub.copy()

def front_min(scan):
    idx = np.arange(-90, 91) % 360
    return float(np.min(scan[idx]))

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

def side_dist(scan, side):
    idx = np.arange(85, 96) % 360 if side == "L" else np.arange(265, 276) % 360
    return float(np.mean(scan[idx]))

def side_min(scan, start, end):
    return float(np.min(scan[np.arange(start, end) % 360]))

def wall_follow(scan, fm, adir, follow_side):
    sd = side_dist(scan, follow_side)
    left_close, right_close = side_min(scan, 60, 120), side_min(scan, 240, 300)
    sign = 1 if follow_side == "L" else -1
    if fm < 18.0: return (0.08, adir * 1.1)
    if fm < 30.0: return (0.10, adir * 0.85)
    if left_close < 18.0: return (0.15, -0.7)
    if right_close < 18.0: return (0.15, 0.7)
    if sd > 40.0: return (0.15, sign * 0.5)
    w = sign * 0.012 * (sd - 20.0)
    return (0.22, np.clip(w, -0.9, 0.9))

# ── CONTROL SMOOTHING ─────────────────────────────────────────────────
prev_v, prev_w = 0.0, 0.0
def send_cmd_smooth(v, w):
    global prev_v, prev_w
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    v = 0.6 * prev_v + 0.4 * v
    w = 0.6 * prev_w + 0.4 * w
    prev_v, prev_w = v, w
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    global prev_v, prev_w
    prev_v, prev_w = 0.0, 0.0
    arduino_ser.write(b"0.000,0.000\n")

# ── COLOR CONFIG & MAIN LOOP (이후 코드 생략) ──────────────────────────
# 위와 같이 함수를 모두 상단에 배치하고 메인 루프를 실행하면 NameError가 해결됩니다.
