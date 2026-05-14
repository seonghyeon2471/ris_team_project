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
    v = max(-0.35, min(0.35, v))
    w = max(-MAX_W, min(MAX_W, w))
    return v, w


# =========================================
# MAIN LOOP
# =========================================
while True:

    scan = lidar.read_scan()

    if scan is None or len(scan) == 0:
        continue

    n = len(scan)

    # =========================================
    # 1. GAP PLANNING
    # =========================================
    angle = planner.find_best_gap(scan)

    # =========================================
    # 2. SMOOTHING
    # =========================================
    angle = 0.7 * prev_angle + 0.3 * angle
    prev_angle = angle

    # =========================================
    # 3. FRONT EMERGENCY CHECK (FIXED)
    # =========================================
    front = scan[:15] + scan[-15:]
    front_dists = [x[0] for x in front]

    if min(front_dists) < SAFE_DIST:
        v = -0.15

        left_sum = sum([x[0] for x in scan[:n//2]])
        right_sum = sum([x[0] for x in scan[n//2:]])

        w = 0.8 if left_sum > right_sum else -0.8

        v, w = clip_speed(v, w)
        motor.send(v, w)
        continue

    # =========================================
    # 4. SPEED CONTROL
    # =========================================
    v = BASE_SPEED * (1 - min(abs(angle), 1.0))
    w = angle

    # =========================================
    # 5. LIMIT
    # =========================================
    v, w = clip_speed(v, w)

    # =========================================
    # 6. MOTOR
    # =========================================
    motor.send(v, w)

    time.sleep(0.02)
