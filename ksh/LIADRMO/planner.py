import math
import numpy as np

from config import *


def footprint_clearance(angle_deg):
    """
    라이다 좌표계에서 특정 방향(angle_deg)으로 장애물이 있을 때,
    로봇 footprint 모서리/면이 닿기 전까지의 '라이다 기준 최소 안전거리(m)'를 반환.

    좌표계 정의
    -----------
    라이다 원점, 전방 = +Y, 우측 = +X (로봇 진행 방향 기준)
    각도 0° = 전방, 양수 = 우측, 음수 = 좌측 (scan 관례)

    footprint (로봇 중심 기준 직사각형)
    ------------------------------------
      전방  : +ROBOT_HALF_LENGTH = 0.10 m
      후방  : -ROBOT_HALF_LENGTH = -0.10 m
      우측  : +ROBOT_HALF_WIDTH  = 0.10 m
      좌측  : -ROBOT_HALF_WIDTH  = -0.10 m

    라이다 → 로봇 중심 오프셋
    --------------------------
      라이다가 중심보다 LIDAR_OFFSET_X = 0.075 m 앞에 있으므로
      로봇 중심은 라이다 기준으로 후방 0.075 m
      → footprint 경계를 라이다 기준으로 변환:
          전방면  : 0.10 - 0.075 = 0.025 m  (라이다 기준 전방)
          후방면  : -0.10 - 0.075 = -0.175 m  (후방 — 진행 중엔 무관)
          좌우면  : ±0.10 m  (라이다 기준 좌우, 오프셋 없음)
    """

    # 라이다 기준 footprint 경계 (m)
    front_edge = ROBOT_HALF_LENGTH - LIDAR_OFFSET_X   # +0.025
    rear_edge  = -ROBOT_HALF_LENGTH - LIDAR_OFFSET_X  # -0.175 (후방)
    side_edge  = ROBOT_HALF_WIDTH                      # ±0.10

    rad = math.radians(angle_deg)
    dx = math.sin(rad)   # 우측 성분
    dy = math.cos(rad)   # 전방 성분

    # 단위 벡터 방향으로 footprint 경계까지의 거리를 ray-box intersection으로 계산
    # 박스: x ∈ [-side_edge, +side_edge], y ∈ [rear_edge, front_edge]
    t_candidates = []

    if abs(dx) > 1e-9:
        t_candidates.append(( side_edge - 0) / dx if dx > 0 else (-side_edge - 0) / dx)
        t_candidates.append((-side_edge - 0) / dx if dx > 0 else ( side_edge - 0) / dx)

    if abs(dy) > 1e-9:
        t_candidates.append((front_edge - 0) / dy if dy > 0 else (rear_edge  - 0) / dy)
        t_candidates.append((rear_edge  - 0) / dy if dy > 0 else (front_edge - 0) / dy)

    # 양수 t 중 최소값이 교차 거리
    t_pos = [t for t in t_candidates if t > 1e-9]
    if not t_pos:
        return 0.0

    clearance = min(t_pos) + FOOTPRINT_MARGIN
    return max(clearance, 0.0)


class GapPlanner:

    def __init__(self):
        # 각도별 안전거리 캐시 (1도 단위, -180~180)
        self._clearance_cache = {
            a: footprint_clearance(a)
            for a in range(-180, 181)
        }

    def _clearance(self, angle_deg):
        key = int(round(angle_deg))
        key = max(-180, min(180, key))
        return self._clearance_cache[key]

    # ------------------------------------------------------------------
    # footprint 보정: 라이다 거리 → 장애물까지 실제 여유거리 (m)
    # ------------------------------------------------------------------

    def obstacle_margin(self, angle_deg, dist_mm):
        """
        각도 angle_deg 방향으로 dist_mm(mm) 위치의 장애물이
        footprint로부터 얼마나 떨어져 있는지 반환 (m).
        음수 = 이미 침범.
        """
        dist_m = dist_mm / 1000.0
        return dist_m - self._clearance(angle_deg)

    # ------------------------------------------------------------------
    # Gap detection (footprint-aware)
    # ------------------------------------------------------------------

    def find_gaps(self, scan):
        """
        footprint 안전거리를 반영한 통과 가능 gap 탐색.
        각 point에서 obstacle_margin > 0 인 경우만 'free' 로 간주.
        """
        gaps = []
        current_gap = []

        for angle, dist in scan:
            margin = self.obstacle_margin(angle, dist)

            # footprint 기준으로 여유 있고, 원시 거리도 threshold 이상
            is_free = (dist > GAP_DISTANCE_THRESHOLD) and (margin > 0)

            if is_free:
                current_gap.append((angle, dist))
            else:
                if len(current_gap) >= MIN_GAP_SIZE:
                    gaps.append(current_gap)
                current_gap = []

        if len(current_gap) >= MIN_GAP_SIZE:
            gaps.append(current_gap)

        return gaps

    # ------------------------------------------------------------------
    # Gap selection
    # ------------------------------------------------------------------

    def select_gap(self, gaps):

        if not gaps:
            return None

        best_score = -999999
        best_angle = 0

        for gap in gaps:

            center_idx = len(gap) // 2
            angle = gap[center_idx][0]
            avg_dist = np.mean([d for a, d in gap])

            score = 0

            # 전방 우선
            score += math.cos(math.radians(angle)) * GOAL_DIRECTION_WEIGHT

            # 넓은 gap 우선 (mm → m 정규화)
            score += avg_dist / 1000.0

            # 중앙 우선
            score -= abs(angle) * 0.01 * CENTER_WEIGHT

            if score > best_score:
                best_score = score
                best_angle = angle

        return best_angle

    # ------------------------------------------------------------------
    # Emergency check (footprint-aware)
    # ------------------------------------------------------------------

    def _emergency_check(self, scan):
        """
        어느 방향이든 footprint 여유거리가 EMERGENCY_DISTANCE 미만이면 True.
        전방 ±90° 범위만 체크 (후방 장애물은 무시).
        """
        for angle, dist in scan:
            if not (-90 <= angle <= 90):
                continue
            margin = self.obstacle_margin(angle, dist)
            if margin < EMERGENCY_DISTANCE:
                return True
        return False

    def _safe_check(self, scan):
        """
        전방 ±30° 범위 footprint 여유거리가 SAFE_DISTANCE 미만이면 True.
        """
        for angle, dist in scan:
            if not (-30 <= angle <= 30):
                continue
            margin = self.obstacle_margin(angle, dist)
            if margin < SAFE_DISTANCE:
                return True
        return False

    # ------------------------------------------------------------------
    # Main control
    # ------------------------------------------------------------------

    def compute_control(self, scan):

        # 1) 비상 정지/회전
        if self._emergency_check(scan):
            return 0.0, 1.8

        # 2) Gap 탐색 및 조향
        gaps = self.find_gaps(scan)
        target_angle = self.select_gap(gaps)

        if target_angle is None:
            # gap 없음 → 제자리 회전
            return 0.0, 1.5

        # steering
        w = -math.radians(target_angle) * 1.4
        if abs(w) < 0.08:
            w = 0.0
        w = max(-MAX_W, min(MAX_W, w))

        # 3) 전방 장애물 접근 시 속도 감소
        speed_scale = 1.0 - min(abs(w) / MAX_W, 0.7)
        if self._safe_check(scan):
            speed_scale *= 0.6   # 접근 중이면 추가 감속

        v = FORWARD_SPEED * speed_scale

        return v, w
