import math
import numpy as np

from config import *

class GapPlanner:

    def __init__(self):
        pass

    # =========================
    # 로봇 footprint 고려
    # =========================

    def required_width(self, angle_deg):

        angle_rad = math.radians(abs(angle_deg))

        # 회전 시 필요한 폭 증가
        dynamic_width = (
            ROBOT_WIDTH * math.cos(angle_rad)
            +
            ROBOT_LENGTH * math.sin(angle_rad)
        )

        return (
            dynamic_width / 2.0
            + SAFETY_MARGIN
        ) * 1000.0

    # =========================
    # GAP 탐색
    # =========================

    def find_gaps(self, scan):

        gaps = []

        current_gap = []

        for angle, dist in scan:

            required = self.required_width(angle)

            # 충분히 넓은 공간만 통과 허용
            if dist > max(GAP_DISTANCE_THRESHOLD, required):

                current_gap.append((angle, dist))

            else:

                if len(current_gap) >= MIN_GAP_SIZE:

                    gaps.append(current_gap)

                current_gap = []

        if len(current_gap) >= MIN_GAP_SIZE:

            gaps.append(current_gap)

        return gaps

    # =========================
    # GAP 선택
    # =========================

    def select_gap(self, gaps):

        if len(gaps) == 0:
            return None

        best_score = -999999
        best_angle = 0

        for gap in gaps:

            center_idx = len(gap) // 2

            angle = gap[center_idx][0]

            avg_dist = np.mean(
                [d for a, d in gap]
            )

            gap_size = len(gap)

            score = 0.0

            # 전방 선호
            score += (
                math.cos(math.radians(angle))
                * GOAL_DIRECTION_WEIGHT
            )

            # 먼 공간 선호
            score += avg_dist / 1200.0

            # 넓은 gap 선호
            score += gap_size * 0.05

            # 급회전 패널티
            score -= (
                abs(angle)
                * 0.012
                * CENTER_WEIGHT
            )

            # 너무 벽 가까우면 감점
            if avg_dist < TURN_CLEARANCE:
                score -= 2.0

            if score > best_score:

                best_score = score
                best_angle = angle

        return best_angle

    # =========================
    # 제어 계산
    # =========================

    def compute_control(self, scan):

        front = []

        left_side = []

        right_side = []

        for angle, dist in scan:

            # 정면
            if -15 <= angle <= 15:
                front.append(dist)

            # 좌측 근접
            if 60 <= angle <= 100:
                left_side.append(dist)

            # 우측 근접
            if -100 <= angle <= -60:
                right_side.append(dist)

        # =========================
        # 비상 회피
        # =========================

        if len(front) > 0:

            front_min = min(front)

            if front_min < EMERGENCY_DISTANCE * 1000:

                # 더 넓은 쪽으로 회전
                left_avg = (
                    np.mean(left_side)
                    if len(left_side) > 0
                    else 9999
                )

                right_avg = (
                    np.mean(right_side)
                    if len(right_side) > 0
                    else 9999
                )

                if left_avg > right_avg:

                    return 0.0, 0.8

                else:

                    return 0.0, -0.8

        # =========================
        # GAP 탐색
        # =========================

        gaps = self.find_gaps(scan)

        target_angle = self.select_gap(gaps)

        # =========================
        # GAP 없음
        # =========================

        if target_angle is None:

            left_avg = (
                np.mean(left_side)
                if len(left_side) > 0
                else 9999
            )

            right_avg = (
                np.mean(right_side)
                if len(right_side) > 0
                else 9999
            )

            if left_avg > right_avg:

                return 0.03, 0.7

            else:

                return 0.03, -0.7

        # =========================
        # Steering
        # =========================

        w = -math.radians(target_angle) * 0.75

        # 작은 진동 제거
        if abs(w) < 0.05:
            w = 0.0

        # 회전 제한
        w = max(-MAX_W, min(MAX_W, w))

        # =========================
        # 회전 시 감속
        # =========================

        speed_scale = (
            1.0
            - min(abs(w) / MAX_W, 0.75)
        )

        v = FORWARD_SPEED * speed_scale

        # 너무 느려지는 거 방지
        v = max(v, MIN_FORWARD_SPEED)

        # =========================
        # 벽 너무 가까우면 감속
        # =========================

        if len(front) > 0:

            front_min = min(front)

            if front_min < SAFE_DISTANCE * 1000:

                v *= 0.5

        return v, w
