#!/usr/bin/env python3
"""
Stable RP LiDAR Obstacle Avoidance (FIXED VERSION)
"""

import time
import math
import serial
import numpy as np
from rplidar import RPLidar

# ─────────────────────────────
# PORT (⚠️ 중요 수정)
# ─────────────────────────────
LIDAR_PORT   = "/dev/ttyUSB0"
ARDUINO_PORT = "/dev/ttyACM0"   # 🔥 반드시 분리 (중요)
ARDUINO_BAUD = 115200

# ─────────────────────────────
# ROBOT
# ─────────────────────────────
ROBOT_WIDTH  = 0.16
ROBOT_LENGTH = 0.20

LIDAR_OFFSET = 0.075
MIN_GAP_WIDTH = ROBOT_WIDTH + 0.10

# ─────────────────────────────
# THRESHOLD
# ─────────────────────────────
OBSTACLE_THRESHOLD  = 0.55
EMERGENCY_STOP_DIST = 0.22

SCAN_HALF_ANGLE = 90
FRONT_ANGLE = 20

MAX_V = 0.18
MAX_W = 1.4
KP_W  = 1.2


# ─────────────────────────────
# UTIL
# ─────────────────────────────
def norm_angle(a):
    a = a % 360
    if a > 180:
        a -= 360
    return a


def parse_scan(scan, max_dist=6.0):

    data = {}

    if len(scan) < 15:   # 🔥 stronger filter
        return data

    for _, angle, dist in scan:

        if dist == 0:
            continue

        d = dist / 1000.0
        if d > max_dist:
            continue

        a = norm_angle(angle)
        a = round(a, 1)

        if a not in data or d < data[a]:
            data[a] = d

    return data


def front_min(scan):
    vals = [d for a, d in scan.items() if abs(a) <= FRONT_ANGLE]
    return min(vals) if vals else float("inf")


# ─────────────────────────────
# GAP DETECTION (FIXED)
# ─────────────────────────────
def find_gaps(scan):

    angles = sorted([a for a in scan if -SCAN_HALF_ANGLE <= a <= SCAN_HALF_ANGLE])
    if not angles:
        return []

    free = {a: scan[a] > (OBSTACLE_THRESHOLD + LIDAR_OFFSET) for a in angles}

    gaps = []
    cur_a, cur_d = [], []
    in_gap = False

    def flush():
        if len(cur_a) < 4:
            return

        span = abs(cur_a[-1] - cur_a[0])
        avg_d = np.mean(cur_d)

        width = 2 * avg_d * math.sin(math.radians(span / 2))

        if width >= MIN_GAP_WIDTH:
            gaps.append({
                "center": np.mean(cur_a),
                "width": width,
                "dist": avg_d
            })

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


# ─────────────────────────────
# GAP SELECTION (FIXED)
# ─────────────────────────────
def select_gap(gaps):

    if not gaps:
        return None

    # 🔥 안정 scoring (음수 제거)
    def score(g):
        return (g["width"] * 2.0) - abs(g["center"]) * 0.8 + g["dist"] * 0.5

    return max(gaps, key=score)


# ─────────────────────────────
# CONTROL
# ─────────────────────────────
def compute_cmd(gap, front_dist):

    ang = math.radians(gap["center"])

    safe_ratio = np.clip(
        (front_dist - EMERGENCY_STOP_DIST) /
        (OBSTACLE_THRESHOLD - EMERGENCY_STOP_DIST),
        0.0, 1.0
    )

    v = MAX_V * safe_ratio * (1.0 - min(abs(ang) / 1.5, 0.7))
    w = KP_W * ang

    w = np.clip(w, -MAX_W, MAX_W)

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

    print("STABLE NAV FIXED START")

    # Arduino
    try:
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        time.sleep(2)
        print("Arduino OK")
    except:
        arduino = None
        print("Arduino FAIL")

    # LiDAR
    lidar = RPLidar(LIDAR_PORT, baudrate=115200, timeout=3)
    lidar.start_motor()
    time.sleep(2)

    print("LiDAR OK")

    try:
        for scan in lidar.iter_scans(min_len=5):

            scan = parse_scan(scan)
            if not scan:
                continue

            front = front_min(scan)
            robot_front = front - LIDAR_OFFSET

            # EMERGENCY
            if robot_front < EMERGENCY_STOP_DIST:
                send(arduino, 0, 0)
                print("[STOP]", robot_front)
                continue

            gaps = find_gaps(scan)
            best = select_gap(gaps)

            if best is None:
                v, w = 0.0, 0.6
            else:
                v, w = compute_cmd(best, front)

            send(arduino, v, w)

            print(f"F:{robot_front*100:5.1f}cm | "
                  f"G:{best['center'] if best else None} | "
                  f"v:{v:.2f} w:{w:.2f}")

    except Exception as e:
        print("ERROR:", e)

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

        print("STOPPED")


if __name__ == "__main__":
    main()
