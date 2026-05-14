import math
import heapq
import numpy as np

from config import *


# ======================================================================
# footprint 안전거리 유틸
# ======================================================================

def footprint_clearance(angle_deg):
    front_edge =  ROBOT_HALF_LENGTH - LIDAR_OFFSET_X
    rear_edge  = -ROBOT_HALF_LENGTH - LIDAR_OFFSET_X
    side_edge  =  ROBOT_HALF_WIDTH

    rad = math.radians(angle_deg)
    dx = math.sin(rad)
    dy = math.cos(rad)

    t_candidates = []
    if abs(dx) > 1e-9:
        t_candidates.append( side_edge / dx if dx > 0 else -side_edge / dx)
        t_candidates.append(-side_edge / dx if dx > 0 else  side_edge / dx)
    if abs(dy) > 1e-9:
        t_candidates.append(front_edge / dy if dy > 0 else rear_edge  / dy)
        t_candidates.append(rear_edge  / dy if dy > 0 else front_edge / dy)

    t_pos = [t for t in t_candidates if t > 1e-9]
    if not t_pos:
        return 0.0
    return min(t_pos) + FOOTPRINT_MARGIN


# ======================================================================
# A* 로컬 경로 탐색
# ======================================================================

def astar(inflated_map, start, goal):
    """
    inflated_map : np.ndarray (MAP_SIZE x MAP_SIZE), 0=free / 1=obstacle
    start, goal  : (cx, cy) 셀 좌표
    반환         : [(cx, cy), ...] 경로 리스트, 실패 시 []
    """
    sx, sy = start
    gx, gy = goal
    size_y, size_x = inflated_map.shape

    def h(x, y):
        return math.hypot(gx - x, gy - y)

    open_heap = []
    heapq.heappush(open_heap, (h(sx, sy), 0.0, sx, sy))

    came_from = {}
    g_score = {(sx, sy): 0.0}
    closed = set()

    neighbors = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
    ]

    while open_heap:
        f, g, cx, cy = heapq.heappop(open_heap)

        if (cx, cy) in closed:
            continue
        closed.add((cx, cy))

        if cx == gx and cy == gy:
            path = []
            node = (cx, cy)
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path

        for dx, dy, cost in neighbors:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or nx >= size_x or ny < 0 or ny >= size_y:
                continue
            if inflated_map[ny, nx] != 0:
                continue
            if (nx, ny) in closed:
                continue

            ng = g + cost
            if ng < g_score.get((nx, ny), float('inf')):
                g_score[(nx, ny)] = ng
                came_from[(nx, ny)] = (cx, cy)
                heapq.heappush(open_heap, (ng + h(nx, ny), ng, nx, ny))

    return []


def smooth_path(path, weight_smooth=0.5, weight_data=0.5, iterations=30):
    """gradient descent 경로 스무딩."""
    if len(path) < 3:
        return path
    p = [list(pt) for pt in path]
    original = [list(pt) for pt in path]
    for _ in range(iterations):
        for i in range(1, len(p) - 1):
            for j in range(2):
                p[i][j] += (
                    weight_data * (original[i][j] - p[i][j]) +
                    weight_smooth * (p[i-1][j] + p[i+1][j] - 2.0 * p[i][j])
                )
    return [(int(round(pt[0])), int(round(pt[1]))) for pt in p]


# ======================================================================
# Gap Planner (비상 회피 전담)
# ======================================================================

class GapPlanner:

    def __init__(self):
        self._clearance_cache = {
            a: footprint_clearance(a) for a in range(-180, 181)
        }

    def _clearance(self, angle_deg):
        key = max(-180, min(180, int(round(angle_deg))))
        return self._clearance_cache[key]

    def obstacle_margin(self, angle_deg, dist_mm):
        return dist_mm / 1000.0 - self._clearance(angle_deg)

    def find_gaps(self, scan):
        gaps, current_gap = [], []
        for angle, dist in scan:
            is_free = (
                dist > GAP_DISTANCE_THRESHOLD and
                self.obstacle_margin(angle, dist) > 0
            )
            if is_free:
                current_gap.append((angle, dist))
            else:
                if len(current_gap) >= MIN_GAP_SIZE:
                    gaps.append(current_gap)
                current_gap = []
        if len(current_gap) >= MIN_GAP_SIZE:
            gaps.append(current_gap)
        return gaps

    def select_gap(self, gaps):
        if not gaps:
            return None
        best_score, best_angle = -999999, 0
        for gap in gaps:
            angle = gap[len(gap) // 2][0]
            avg_dist = np.mean([d for _, d in gap])
            score = (
                math.cos(math.radians(angle)) * GOAL_DIRECTION_WEIGHT
                + avg_dist / 1000.0
                - abs(angle) * 0.01 * CENTER_WEIGHT
            )
            if score > best_score:
                best_score, best_angle = score, angle
        return best_angle

    def emergency_steer(self, scan):
        """비상 상황 조향각(deg) 반환. 위험 없으면 None."""
        for angle, dist in scan:
            if not (-90 <= angle <= 90):
                continue
            if self.obstacle_margin(angle, dist) < EMERGENCY_DISTANCE:
                gaps = self.find_gaps(scan)
                return self.select_gap(gaps) or 45.0
        return None


# ======================================================================
# 로컬 경로 플래너 (맵 기반 A* + gap 비상 회피 혼합)
# ======================================================================

class LocalPathPlanner:
    """
    mapper.inflated 위에서 A* 로 로컬 웨이포인트를 결정하고,
    그 방향으로 diff-drive 속도 명령을 생성.
    비상 상황은 GapPlanner가 오버라이드.
    """

    LOOKAHEAD_M      = 0.50   # A* 목표까지 거리 (m) — 0.60 → 0.50 (맵 여유 확보)
    WP_ARRIVAL_M     = 0.12   # 웨이포인트 도착 판정 (m)
    REPLAN_INTERVAL  = 4      # 8 → 4: 초기 경로를 더 빠르게 잡음

    def __init__(self):
        self._gap = GapPlanner()
        self._waypoints = []
        self._wp_idx = 0
        self._loop_count = 0

    # ------------------------------------------------------------------
    # 목표 셀 선택
    # ------------------------------------------------------------------

    def _make_goal(self, mapper):
        rx, ry = mapper.robot_cell()
        lookahead_cells = int(self.LOOKAHEAD_M / MAP_RESOLUTION)

        best_score, best_cell = -999999, None

        # 각도 범위 -60~60 → -80~80 으로 확대 (좁은 공간에서 목표를 못 찾는 경우 방지)
        for angle_deg in range(-80, 81, 5):
            rad = math.radians(angle_deg)
            cx = int(rx + lookahead_cells * math.sin(rad))
            cy = int(ry - lookahead_cells * math.cos(rad))

            if not mapper.is_free(cx, cy):
                continue

            fwd_score    =  math.cos(rad) * GOAL_DIRECTION_WEIGHT
            visit_score  = -mapper.visit[cy, cx] * VISIT_WEIGHT * 0.01
            center_score = -abs(angle_deg) * 0.01 * CENTER_WEIGHT

            score = fwd_score + visit_score + center_score
            if score > best_score:
                best_score, best_cell = score, (cx, cy)

        # lookahead 거리에서 못 찾으면 절반 거리로 재시도
        if best_cell is None:
            short_cells = max(1, lookahead_cells // 2)
            for angle_deg in range(-80, 81, 5):
                rad = math.radians(angle_deg)
                cx = int(rx + short_cells * math.sin(rad))
                cy = int(ry - short_cells * math.cos(rad))

                if not mapper.is_free(cx, cy):
                    continue

                fwd_score    =  math.cos(rad) * GOAL_DIRECTION_WEIGHT
                visit_score  = -mapper.visit[cy, cx] * VISIT_WEIGHT * 0.01
                center_score = -abs(angle_deg) * 0.01 * CENTER_WEIGHT

                score = fwd_score + visit_score + center_score
                if score > best_score:
                    best_score, best_cell = score, (cx, cy)

        if best_cell is None:
            print("[WARN] _make_goal: 자유 셀을 찾지 못했습니다. 맵이 전부 막혀 있을 수 있습니다.")

        return best_cell

    # ------------------------------------------------------------------
    # 재계획
    # ------------------------------------------------------------------

    def _replan(self, mapper):
        rx, ry = mapper.robot_cell()
        goal = self._make_goal(mapper)

        if goal is None:
            self._waypoints = []
            self._wp_idx = 0
            return

        raw_path = astar(mapper.inflated, (rx, ry), goal)

        if not raw_path:
            print(f"[WARN] A* 경로 없음: start=({rx},{ry}) goal={goal}")
            self._waypoints = []
            self._wp_idx = 0
            return

        self._waypoints = smooth_path(raw_path)
        self._wp_idx = 1   # 0번은 현재 위치

    # ------------------------------------------------------------------
    # 현재 추종 웨이포인트 방향각(deg) 계산
    # ------------------------------------------------------------------

    def _waypoint_angle(self, mapper):
        if not self._waypoints or self._wp_idx >= len(self._waypoints):
            return None

        rx, ry = mapper.robot_cell()
        wp = self._waypoints[self._wp_idx]

        # 도착 판정
        dist_cells = math.hypot(wp[0] - rx, wp[1] - ry)
        arrival_cells = self.WP_ARRIVAL_M / MAP_RESOLUTION
        if dist_cells < arrival_cells:
            self._wp_idx += 1
            if self._wp_idx >= len(self._waypoints):
                return None
            wp = self._waypoints[self._wp_idx]

        # 셀 좌표 → 로봇 기준 각도
        # 맵 y 증가 = 아래 = 로봇 후방
        dy_cells =  ry - wp[1]   # 전방이 양수
        dx_cells =  wp[0] - rx   # 우측이 양수
        return math.degrees(math.atan2(dx_cells, dy_cells))

    # ------------------------------------------------------------------
    # 메인 인터페이스
    # ------------------------------------------------------------------

    def compute_control(self, scan, mapper):
        """
        scan   : [(angle_deg, dist_mm), ...]
        mapper : LocalMapper (update 완료된 상태)
        반환   : (v, w)
        """
        self._loop_count += 1

        # ── 1) 비상 회피 ────────────────────────────────────────────
        emg_angle = self._gap.emergency_steer(scan)
        if emg_angle is not None:
            w = -math.radians(emg_angle) * 1.4
            w = max(-MAX_W, min(MAX_W, w))
            return 0.0, w

        # ── 2) 재계획 ───────────────────────────────────────────────
        needs_replan = (
            self._loop_count % self.REPLAN_INTERVAL == 0 or
            not self._waypoints or
            self._wp_idx >= len(self._waypoints)
        )
        if needs_replan:
            self._replan(mapper)

        # ── 3) 웨이포인트 추종 ──────────────────────────────────────
        target_angle = self._waypoint_angle(mapper)

        if target_angle is None:
            # 경로 없음 → gap planner fallback
            gaps = self._gap.find_gaps(scan)
            target_angle = self._gap.select_gap(gaps)
            if target_angle is None:
                return 0.0, 1.5   # 완전히 막힘 → 제자리 회전

        # ── 4) 속도 계산 ────────────────────────────────────────────
        w = -math.radians(target_angle) * 1.4
        if abs(w) < 0.08:
            w = 0.0
        w = max(-MAX_W, min(MAX_W, w))

        speed_scale = 1.0 - min(abs(w) / MAX_W, 0.7)

        # 전방 근접 장애물 → 추가 감속
        for angle, dist in scan:
            if -30 <= angle <= 30:
                if self._gap.obstacle_margin(angle, dist) < SAFE_DISTANCE:
                    speed_scale *= 0.6
                    break

        v = FORWARD_SPEED * speed_scale
        return v, w

    def get_path(self):
        """디버그: 현재 경로와 웨이포인트 인덱스 반환."""
        return self._waypoints, self._wp_idx
