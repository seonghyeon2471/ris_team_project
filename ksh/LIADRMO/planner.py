import math
import numpy as np
from config import *

class GapPlanner:

    def __init__(self):
        pass

    def compute_control(self, scan):

        front = [d for a, d in scan if -15 <= a <= 15]

        # emergency
        if front and min(front) < EMERGENCY_DISTANCE * 1000:
            return 0.0, 1.8

        # gap 기반 선택
        gaps = self.find_gaps(scan)
        target_angle = self.select_gap(gaps)

        if target_angle is None:
            return 0.0, 1.5

        # ===== FGM + 개선 =====

        w = -math.radians(target_angle) * 1.6

        # 🔥 dead-zone 제거 (벽 붙으면 탈출 강화)
        if abs(target_angle) < 10:
            w += 0.4 * (1 if target_angle > 0 else -1)

        w = max(-MAX_W, min(MAX_W, w))

        speed_scale = 1.0 - min(abs(w) / MAX_W, 0.75)
        v = FORWARD_SPEED * speed_scale

        return v, w
