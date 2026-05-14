import math
import numpy as np
from config import *

class GapPlanner:

    def __init__(self):
        pass

    # =========================
    # GAP DETECTION
    # =========================
    def find_gaps(self, scan):

        gaps = []
        current = []

        for a, d in scan:

            if d > GAP_DISTANCE_THRESHOLD:
                current.append((a, d))
            else:
                if len(current) >= MIN_GAP_SIZE:
                    gaps.append(current)
                current = []

        if len(current) >= MIN_GAP_SIZE:
            gaps.append(current)

        return gaps

    # =========================
    # GRP SCORING
    # =========================
    def select_gap(self, gaps):

        if not gaps:
            return None

        best_score = -1e9
        best_angle = 0

        for gap in gaps:

            center = gap[len(gap)//2][0]
            avg_dist = np.mean([d for _, d in gap])

            # GRP-style scoring
            score = 0

            # forward bias
            score += math.cos(math.radians(center)) * GOAL_BIAS

            # wide gap preference
            score += avg_dist / 1000.0

            # center stability
            score -= abs(center) * 0.01 * CENTER_BIAS

            if score > best_score:
                best_score = score
                best_angle = center

        return best_angle

    # =========================
    # MAIN CONTROL
    # =========================
    def compute_control(self, scan):

        front = [d for a, d in scan if -15 <= a <= 15]

        # emergency
        if front and min(front) < EMERGENCY_DISTANCE:
            return 0.0, 1.8

        gaps = self.find_gaps(scan)
        target = self.select_gap(gaps)

        if target is None:
            return 0.0, 1.5

        # steering
        w = -math.radians(target) * 1.2

        # dead-end escape boost
        if abs(target) < 10:
            w += 0.5 if target > 0 else -0.5

        w = max(-MAX_W, min(MAX_W, w))

        speed = FORWARD_SPEED * (1.0 - min(abs(w)/MAX_W, 0.7))

        return speed, w
