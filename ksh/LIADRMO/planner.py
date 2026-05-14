import math

class GapPlanner:
    def __init__(self):
        # =========================
        # ROBOT PARAMETER
        # =========================
        self.ROBOT_HALF_WIDTH = 0.12
        self.SAFE_MARGIN = 0.10

        # 최소 전방 거리
        self.SAFE_DIST = 0.35

    # =========================================
    # GAP FINDING (SAFE VERSION)
    # =========================================
    def find_best_gap(self, scan):

        n = len(scan)

        best_score = -999
        best_angle = 0

        step = 5  # sampling speed

        for i in range(0, n, step):

            left_idx = (i - 10) % n
            right_idx = (i + 10) % n

            # =========================================
            # 🚨 핵심 수정: tuple → distance만 사용
            # =========================================
            front_dist = scan[i][0]
            left_dist = scan[left_idx][0]
            right_dist = scan[right_idx][0]

            # =========================================
            # 1. FRONT SAFE CHECK
            # =========================================
            if front_dist < self.SAFE_DIST:
                continue

            # =========================================
            # 2. SIDE WIDTH CHECK (핵심)
            # =========================================
            if left_dist < self.ROBOT_HALF_WIDTH + self.SAFE_MARGIN:
                continue

            if right_dist < self.ROBOT_HALF_WIDTH + self.SAFE_MARGIN:
                continue

            # =========================================
            # 3. SCORING (safe + forward bias)
            # =========================================
            center_bias = -abs(i - n // 2) / n

            score = front_dist + center_bias * 0.5

            # =========================================
            # 4. BEST SELECT
            # =========================================
            if score > best_score:
                best_score = score
                best_angle = i

        # =========================================
        # angle conversion
        # =========================================
        angle = (best_angle - n / 2) * (2 * math.pi / n)

        return angle
