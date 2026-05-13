import serial
import time
import threading
import numpy as np

# =========================
# UART
# =========================
arduino = serial.Serial("/dev/serial0", 115200, timeout=1)
ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=1)

# =========================
# PARAMETER
# =========================
SAFE_DIST = 0.30        # 🔥 벽 안 붙게 (중요)
ALPHA = 0.55
TURN_GAIN = 6.0         # 너무 크면 흔들림 → 안정형

# 속도 (요청 반영: 기존보다 낮춤)
v_max = 0.09

# =========================
# DATA
# =========================
scan_data = []
lock = threading.Lock()

# =========================
# LIDAR INIT
# =========================
def init_lidar():
    ser.write(bytes([0xA5, 0x40]))
    time.sleep(2)
    ser.write(bytes([0xA5, 0x20]))

# =========================
# LIDAR THREAD
# =========================
def lidar_loop():

    global scan_data

    while True:
        data = ser.read(5)
        if len(data) != 5:
            continue

        if (data[1] & 0x01) != 1:
            continue

        quality = data[0] >> 2
        if quality < 15:
            continue

        angle_q6 = ((data[1] >> 1) | (data[2] << 7))
        angle = angle_q6 / 64.0

        dist = (data[3] | (data[4] << 8)) / 4.0 / 10.0  # cm

        if dist < 5 or dist > 300:
            continue

        angle = (angle + 180) % 360 - 180

        with lock:
            scan_data.append((angle, dist))

            if len(scan_data) > 720:
                scan_data.pop(0)

# =========================
# FRONT SCAN (120°)
# =========================
def get_front_scan():
    with lock:
        scan = list(scan_data)

    return [(a, d) for a, d in scan if -60 <= a <= 60]

# =========================
# SPEED CONTROL
# =========================
def speed_control(scan):

    if len(scan) == 0:
        return 0

    min_dist = min([d for _, d in scan])

    if min_dist < 0.20:
        return 0.04
    elif min_dist < 0.35:
        return 0.06
    else:
        return v_max

# =========================
# GAP DETECTION (핵심 개선)
# =========================
def find_gaps(scan):

    scan = sorted(scan, key=lambda x: x[0])

    gaps = []
    temp = []

    for a, d in scan:

        if d > SAFE_DIST:
            temp.append((a, d))

        else:
            if len(temp) > 4:   # 🔥 너무 작은 gap 제거
                gaps.append(temp)
            temp = []

    if len(temp) > 4:
        gaps.append(temp)

    return gaps

# =========================
# GAP SCORE (핵심)
# =========================
def score_gap(gap):

    angles = [a for a, _ in gap]
    dists  = [d for _, d in gap]

    width = max(angles) - min(angles)
    avg_dist = sum(dists) / len(dists)

    return width * avg_dist

# =========================
# BEST GAP
# =========================
def select_gap(gaps):

    if not gaps:
        return None

    best = max(gaps, key=score_gap)

    center = sum([a for a, _ in best]) / len(best)

    return center

# =========================
# GRP CONTROLLER
# =========================
def compute_grp(scan):

    gaps = find_gaps(scan)

    v = speed_control(scan)

    # =========================
    # CASE 1: gap 있음
    # =========================
    if gaps:

        gap_angle = select_gap(gaps)
        ref_angle = 0.0

        theta = ALPHA * ref_angle + (1 - ALPHA) * gap_angle

        w = np.radians(theta * TURN_GAIN)

        return v, w

    # =========================
    # CASE 2: gap 없음 (fallback)
    # =========================
    min_dist = min([d for _, d in scan], default=999)

    # 완전 막힘 → 제자리 회전
    if min_dist < 0.25:
        return 0.0, 0.8

    # 애매 → 약한 회전
    return 0.03, np.radians(25)

# =========================
# SEND
# =========================
def send(v, w):
    arduino.write(f"{v},{w}\n".encode())

# =========================
# MAIN
# =========================
print("STABLE GRP START")

init_lidar()

t = threading.Thread(target=lidar_loop, daemon=True)
t.start()

try:
    while True:

        scan = get_front_scan()

        if len(scan) < 10:
            send(0, 0)
            continue

        v, w = compute_grp(scan)

        send(v, w)

        print(f"v={v:.2f}, w={w:.2f}, pts={len(scan)}")

        time.sleep(0.05)

except KeyboardInterrupt:
    send(0, 0)
    arduino.close()
    ser.close()
    print("STOP")
