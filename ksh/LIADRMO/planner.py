import math
import numpy as np
from config import *

class GapPlanner:

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

    def select_gap(self, gaps):

        if not gaps:
            return None

        best_score = -1e9
        best_angle = 0

        for gap in gaps:

            center = gap[len(gap)//2][0]
            avg_dist = np.mean([d for _, d in gap])

            score = 0
            score += math.cos(math.radians(center)) * 2.0
            score += avg_dist / 1000.0
            score -= abs(center) * 0.01 * 0.8

            if score > best_score:
                best_score = score
                best_angle = center

        return best_angle

    # =========================
    # MAIN CONTROL
    # =========================
    def compute_control(self, scan):

        # =========================
        # EMERGENCY
        # =========================
        front = [d for a, d in scan if -15 <= a <= 15]

        if front and min(front) < EMERGENCY_DISTANCE:
            return 0.0, 1.8

        # =========================
        # 🔥 NARROW PASSAGE MODE (30cm)
        # =========================
        left = [d for a, d in scan if -90 <= a < -10]
        right = [d for a, d in scan if 10 < a <= 90]

        if len(left) > 5 and len(right) > 5:

            l = np.mean(left)
            r = np.mean(right)

            error = r - l

            w = error * 0.003

            # 직진 안정 보정
            w += -math.radians(np.mean([a for a, _ in scan if -10 <= a <= 10])) * 0.5

            speed = FORWARD_SPEED * 0.75

            w = max(-1.5, min(1.5, w))

            return speed, w

        # =========================
        # NORMAL GAP MODE
        # =========================
        gaps = self.find_gaps(scan)
        target = self.select_gap(gaps)

        if target is None:
            return 0.0, 1.5

        w = -math.radians(target) * 1.2

        if abs(target) < 10:
            w += 0.5 if target > 0 else -0.5

        left_all = [d for a, d in scan if -90 <= a < 0]
        right_all = [d for a, d in scan if 0 < a <= 90]

        if left_all and right_all:
            w += (np.mean(right_all) - np.mean(left_all)) * 0.002

        w = max(-MAX_W, min(MAX_W, w))

        speed = FORWARD_SPEED * (1.0 - min(abs(w)/MAX_W, 0.7))

        return speed, w
