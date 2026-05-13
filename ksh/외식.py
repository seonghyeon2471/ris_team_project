#!/usr/bin/env python3
"""
Stable RP LiDAR Obstacle Avoidance (Raspberry Pi + Arduino)
- Descriptor error 방지
- Gap 기반 주행
- 자동 복구 포함
"""

import time
import math
import serial
import numpy as np
from rplidar import RPLidar

# ─────────────────────────────
# PORT
# ─────────────────────────────
LIDAR_PORT   = "/dev/ttyUSB0"
ARDUINO_PORT = "/dev/ttyUSB0"   # ⚠️ 필요시 /dev/ttyACM0로 변경
ARDUINO_BAUD = 115200

# ─────────────────────────────
# ROBOT PARAMS
# ─────────────────────────────
ROBOT_WIDTH  = 0.16
ROBOT_LENGTH = 0.20

LIDAR_OFFSET = 0.075
MIN_GAP_WIDTH = ROBOT_WIDTH + 0.10

# ─────────────────────────────
# THRESHOLDS
# ─────────────────────────────
OBSTACLE_THRESHOLD  = 0.55
EMERGENCY_STOP_DIST = 0.22

# ─────────────────────────────
# SCAN CONFIG
# ─────────────────────────────
SCAN_HALF_ANGLE = 90
FRONT_ANGLE = 20

# ─────────────────────────────
# CONTROL
# ─────────────────────────────
MAX_V = 0.18
MAX_W = 1.4
KP_W  = 1.3


# ─────────────────────────────
# UTIL
# ─────────────────────────────
def norm_angle(a):
    a = a % 360
    if a > 180:
        a -= 360
    return a


def parse_scan(scan, max_dist=6.0):
    """LiDAR raw → angle-distance map"""
    data = {}

    if len(scan) < 10:   # 🔥 noise filter
        return data

    for _, angle, dist in scan:
        if dist == 0:
            continue

        d = dist / 1000.0
        if d > max_dist:
            continue

        a = round(norm_angle(angle), 1)

        if a not in data or d < data[a]:
            data[a] = d

    return data


def front_min(scan):
    vals = [d for a, d in scan.items() if abs(a) <= FRONT_ANGLE]
    return min(vals) if vals else float("inf")


def find_gaps(scan):
    angles = sorted([a for a in scan if -SCAN_HALF_ANGLE <= a <= SCAN_HALF_ANGLE])
    if not angles:
        return []

    free = {a: (scan[a] > OBSTACLE_THRESHOLD + LIDAR_OFFSET) for a in angles}

    gaps = []
    cur_a, cur_d = [], []

    def flush():
        if len(cur_a) < 3:
            return

        span = abs(cur_a[-1] - cur_a[0])
        avg_d = np.mean(cur_d)

        width = 2 * avg_d * math.sin(math.radians(span / 2))

        if width >= MIN_GAP_WIDTH:
            gaps.append({
                "center": (cur_a[0] + cur_a[-1]) / 2,
                "width": width,
                "dist": avg_d
            })

    in_gap = False

    for a in angles:
        if free[a]:
            if not in_gap:
                in_gap = True
                cur_a, cur_d = [a], [scan[a]]
            else:
                cur_a.append(a)
                cur_d.append(scan[a])
        else:
            if in_gap:
                flush()
                in_gap = False

    if in_gap:
        flush()

    return gaps


def select_gap(gaps):
    if not gaps:
        return None

    # 🔥 개선된 scoring (정면 + 넓은 gap + 가까운 거리 안정성)
    return min(gaps, key=lambda g: abs(g["center"]) * 1.0 - g["width"] * 6.0)


def compute_cmd(gap, front_dist):
    ang = math.radians(gap["center"])

    safe_ratio = max(min((front_dist - EMERGENCY_STOP_DIST) /
                         (OBSTACLE_THRESHOLD - EMERGENCY_STOP_DIST), 1.0), 0.0)

    v = MAX_V * safe_ratio * (1.0 - min(abs(ang) / 1.5, 0.7))
    w = KP_W * ang

    w = max(-MAX_W, min(MAX_W, w))

    return v, w


def send(ser, v, w):
    if ser is None:
        return
    try:
        ser.write(f"{v:.3f},{w:.3f}\n".encode())
    except:
        pass


# ─────────────────────────────
# MAIN
# ─────────────────────────────
def main():
    print("=" * 50)
    print(" RP LiDAR Stable Navigation Start")
    print("=" * 50)

    # Arduino
    try:
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        time.sleep(2)
        print(f"Arduino OK: {ARDUINO_PORT}")
    except:
        print("Arduino FAIL (continue without it)")
        arduino = None

    # LiDAR
    lidar = RPLidar(LIDAR_PORT, baudrate=115200, timeout=3)
    lidar.start_motor()
    time.sleep(2)
    print("LiDAR OK")

    prev_v, prev_w = 0, 0

    try:
        for scan in lidar.iter_scans(max_buf_meas=100):

            scan = parse_scan(scan)
            if not scan:
                continue

            front = front_min(scan)
            robot_front = front - LIDAR_OFFSET

            # ── EMERGENCY ──
            if robot_front < EMERGENCY_STOP_DIST:
                send(arduino, 0, 0)
                print("[EMERGENCY]", robot_front)
                continue

            gaps = find_gaps(scan)
            best = select_gap(gaps)

            # ── NO GAP ──
            if best is None:
                v, w = 0, MAX_W * 0.5

            else:
                v, w = compute_cmd(best, front)

            send(arduino, v, w)

            print(f"F:{robot_front*100:5.1f}cm | "
                  f"G:{best['center'] if best else None} | "
                  f"v:{v:.2f} w:{w:.2f}")

            prev_v, prev_w = v, w

    except Exception as e:
        print("ERROR:", e)

        # 🔥 auto recovery
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass

    finally:
        send(arduino, 0, 0)

        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass

        if arduino:
            arduino.close()

        print("Stopped")


if __name__ == "__main__":
    main()
