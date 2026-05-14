import numpy as np
import math

# =====================================================
# PARAMETERS
# =====================================================
MAX_SPEED = 0.22
MAX_W = 1.2

FRONT_RANGE = 75
DT = 0.05

ROBOT_RADIUS = 0.10
SAFE_MARGIN = 0.05

EMERGENCY_DIST = 0.12

# rollout
ROLLOUT_TIME = 0.9
ROLLOUT_STEPS = int(ROLLOUT_TIME / DT)

# steering samples
STEER_CANDIDATES = np.linspace(-1.0, 1.0, 21)

# costs
GOAL_GAIN = 2.2
CLEAR_GAIN = 1.6
CENTER_GAIN = 0.8
EDGE_PENALTY = 2.0
TURN_PENALTY = 0.5

# speed
MIN_SPEED = 0.06


# =====================================================
# STATE
# =====================================================
class FTGState:

    def __init__(self):

        self.prev_w = 0.0
        self.stuck_count = 0
        self.prev_goal_dist = 999.0


# =====================================================
# SAFE DISTANCE
# =====================================================
def safe_distance(scan, angle_deg):

    idx = int(angle_deg) % 360

    return float(scan[idx])


# =====================================================
# COLLISION CHECK
# =====================================================
def collision(scan, rel_angle, dist):

    d = safe_distance(scan, rel_angle)

    return d < dist


# =====================================================
# GAP WIDTH ESTIMATE
# =====================================================
def gap_width(scan, angle_deg):

    width = 0

    for a in range(angle_deg - 12, angle_deg + 13):

        if safe_distance(scan, a) > 0.40:
            width += 1

    return width


# =====================================================
# TRAJECTORY SCORE
# =====================================================
def evaluate_trajectory(
        scan,
        goal_angle,
        w_cmd):

    x = 0.0
    y = 0.0
    th = 0.0

    min_clearance = 999.0

    for _ in range(ROLLOUT_STEPS):

        v = MAX_SPEED

        th += w_cmd * DT

        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT

        rel_deg = int(math.degrees(th))

        clearance = safe_distance(scan, rel_deg)

        min_clearance = min(min_clearance, clearance)

        # collision predict
        if clearance < ROBOT_RADIUS + SAFE_MARGIN:
            return -9999

    final_angle = math.degrees(th)

    # goal alignment
    goal_score = math.exp(
        -0.04 * abs(goal_angle - final_angle)
    )

    # center preference
    center_score = math.exp(
        -0.015 * abs(final_angle)
    )

    # corridor width
    width = gap_width(scan, int(final_angle))

    width_score = width / 25.0

    # edge penalty
    edge_cost = 0.0

    if min_clearance < 0.25:
        edge_cost = EDGE_PENALTY * (0.25 - min_clearance)

    score = (
        GOAL_GAIN * goal_score +
        CLEAR_GAIN * width_score +
        CENTER_GAIN * center_score -
        edge_cost -
        TURN_PENALTY * abs(w_cmd)
    )

    return score


# =====================================================
# MAIN PLANNER
# =====================================================
def get_gap_navigation_v11(
        scan_robot,
        goal_angle,
        goal_distance,
        state):

    # =================================================
    # emergency brake
    # =================================================
    front = []

    for a in range(-15, 16):
        front.append(safe_distance(scan_robot, a))

    if np.min(front) < EMERGENCY_DIST:

        return 0.0, 0.0, {
            "mode": "EMERGENCY"
        }

    # =================================================
    # progress detector
    # =================================================
    progress = state.prev_goal_dist - goal_distance

    if progress < 0.01:
        state.stuck_count += 1
    else:
        state.stuck_count = 0

    state.prev_goal_dist = goal_distance

    # =================================================
    # deadlock recovery
    # =================================================
    if state.stuck_count > 25:

        state.stuck_count = 0

        return 0.0, 0.9, {
            "mode": "RECOVERY"
        }

    # =================================================
    # steering rollout search
    # =================================================
    best_score = -999999
    best_w = 0.0

    for w in STEER_CANDIDATES:

        score = evaluate_trajectory(
            scan_robot,
            goal_angle,
            w
        )

        if score > best_score:

            best_score = score
            best_w = w

    # =================================================
    # adaptive speed
    # =================================================
    v = MAX_SPEED * (1.0 - abs(best_w) / MAX_W * 0.5)

    v = float(np.clip(v, MIN_SPEED, MAX_SPEED))

    # =================================================
    # steering smoothing
    # =================================================
    w = (
        0.65 * state.prev_w +
        0.35 * best_w
    )

    state.prev_w = w

    return v, w, {
        "mode": "ROLLOUT",
        "score": round(best_score, 2)
    }
