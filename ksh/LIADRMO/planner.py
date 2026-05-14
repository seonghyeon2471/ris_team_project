import math

class GapPlanner:
    def __init__(self):
        # ===== 로봇 파라미터 =====
        self.ROBOT_HALF_WIDTH = 0.12   # 24cm 로봇 기준
        self.SAFE_MARGIN = 0.10

        # ===== 안전 거리 =====
        self.SAFE_DIST = 0.35

    # =========================================
    # gap 선택 핵심 로직
    # =========================================
    def find_best_gap(self, scan):
        """
        scan: LiDAR 360 distance list
        return: steering angle (rad)
        """

        n = len(scan)
        best_score = -999
        best_angle = 0

        # 10도 단위로 스캔 (속도 vs 안정성)
        step = 5

        for i in range(0, n, step):

            # 좌/우 샘플
            left_idx = (i - 10) % n
            right_idx = (i + 10) % n

            left_dist = scan[left_idx]
            right_dist = scan[right_idx]
            front_dist = scan[i]

            # =========================================
            # 1. 너무 가까운 벽 컷
            # =========================================
            if front_dist < self.SAFE_DIST:
                continue

            # =========================================
            # 2. 측면 안전 체크 (핵심)
            # =========================================
            if left_dist < self.ROBOT_HALF_WIDTH + self.SAFE_MARGIN:
                continue

            if right_dist < self.ROBOT_HALF_WIDTH + self.SAFE_MARGIN:
                continue

            # =========================================
            # 3. scoring (중앙 + 거리)
            # =========================================
            center_bias = -abs(i - n//2) / n
            score = front_dist + center_bias * 0.5

            # =========================================
            # 4. best update
            # =========================================
            if score > best_score:
                best_score = score
                best_angle = i

        # angle normalize (-pi ~ pi)
        angle = (best_angle - n/2) * (2 * math.pi / n)

        return angle
