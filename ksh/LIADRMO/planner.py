import math
import numpy as np

from config import *

class GapPlanner:

    def __init__(self):
        pass

    def find_gaps(self, scan):

        gaps = []

        current_gap = []

        for angle, dist in scan:

            if dist > GAP_DISTANCE_THRESHOLD:

                current_gap.append((angle, dist))

            else:

                if len(current_gap) >= MIN_GAP_SIZE:
                    gaps.append(current_gap)

                current_gap = []

        if len(current_gap) >= MIN_GAP_SIZE:
            gaps.append(current_gap)

        return gaps

    def select_gap(self, gaps):

        if len(gaps) == 0:
            return None

        best_score = -999999
        best_angle = 0

        for gap in gaps:

            center_idx = len(gap) // 2

            angle = gap[center_idx][0]

            avg_dist = np.mean([d for a, d in gap])

            score = 0

            # 전방 우선
            score += (
                math.cos(math.radians(angle))
                * GOAL_DIRECTION_WEIGHT
            )

            # 넓은 gap 우선
            score += avg_dist / 1000.0

            # 중앙 우선
            score -= abs(angle) * 0.01 * CENTER_WEIGHT

            if score > best_score:

                best_score = score
                best_angle = angle

        return best_angle

    def compute_control(self, scan):

        front = []

        for angle, dist in scan:

            if -15 <= angle <= 15:
                front.append(dist)

        if len(front) > 0:

            front_min = min(front)

            # 비상 회피
            if front_min < EMERGENCY_DISTANCE * 1000:

                return 0.0, 1.8

        gaps = self.find_gaps(scan)

        target_angle = self.select_gap(gaps)

        if target_angle is None:

            return 0.0, 1.5

        # steering
        w = -math.radians(target_angle) * 0.8

        if abs(w) < 0.08:
            w = 0.0

        w = max(-MAX_W, min(MAX_W, w))

        # 속도 감소
        speed_scale = 1.0 - min(abs(w) / MAX_W, 0.7)

        v = FORWARD_SPEED * speed_scale

        return v, w
