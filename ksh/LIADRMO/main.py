import time
import math

from lidar import SimpleLidar
from motor import MotorController
from planner import GapPlanner

# =========================================
# INIT
# =========================================
lidar = SimpleLidar()
motor = MotorController()
planner = GapPlanner()

print("SYSTEM START")

# =========================================
# PARAMETERS
# =========================================
SAFE_DIST = 0.35
BASE_SPEED = 0.25

prev_angle = 0

# =========================================
# SPEED LIMIT
# =========================================
def clip_speed(v, w):
    MAX_W = 0.8

    if abs(w) > MAX_W:
        w = MAX_W if w > 0 else -MAX_W

    return v, w


# =========================================
# MAIN LOOP
# =========================================
while True:

    scan = lidar.read_scan()

    if scan is None or len(scan) == 0:
        continue

    # -----------------------------------------
    # 1. GAP PLANNING
    # -----------------------------------------
    angle = planner.find_best_gap(scan)

    # -----------------------------------------
    # 2. SMOOTHING (핵심 안정화)
    # -----------------------------------------
    angle = 0.7 * prev_angle + 0.3 * angle
    prev_angle = angle

    # -----------------------------------------
    # 3. EMERGENCY FRONT CHECK
    # -----------------------------------------
    n = len(scan)
    front = scan[:15] + scan[-15:]

    if min(front) < SAFE_DIST:
        # 무조건 회피 (후진 + 회전)
        v = -0.15
        w = 0.8 if sum(scan[:n//2]) > sum(scan[n//2:]) else -0.8

        v, w = clip_speed(v, w)
        motor.send(v, w)
        continue

    # -----------------------------------------
    # 4. SPEED CONTROL (angle 기반 감속)
    # -----------------------------------------
    v = BASE_SPEED * (1 - min(abs(angle), 1.0))

    # steering
    w = angle

    # -----------------------------------------
    # 5. LIMITS
    # -----------------------------------------
    v, w = clip_speed(v, w)

    # -----------------------------------------
    # 6. SEND TO MOTOR
    # -----------------------------------------
    motor.send(v, w)

    time.sleep(0.02)
