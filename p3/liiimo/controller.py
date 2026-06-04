"""
Pure Pursuit Controller — 차동구동 버전
========================================
실제 파라미터:
  바퀴 반지름 = 0.034 m  →  둘레 ≈ 0.2136 m
  트랙 폭(좌우 간격) = 실측 필요 (기본 0.18 m)

출력: 좌/우 모터 속도 (m/s) + 등가 조향각 (도, HUD용)

Reference: arXiv:2507.12449 §II-B2
"""

import logging
import math
import numpy as np

log = logging.getLogger("pure_pursuit")

WHEEL_RADIUS = 0.034
WHEEL_CIRC   = 2 * math.pi * WHEEL_RADIUS   # ≈ 0.2136 m/rev


class PurePursuitController:
    def __init__(self, cfg: dict):
        # 트랙 폭 (좌우 바퀴 간격) — wheelbase 키 재활용
        self.track_width = cfg["wheelbase"]
        self.ld          = cfg["lookahead_dist"]
        self.max_steer   = math.radians(cfg["max_steer_deg"])

    # ──────────────────────────────────────────────────
    def compute(self,
                pos: np.ndarray,    # (2,) 차량 위치
                yaw: float,         # 헤딩 (rad)
                path: np.ndarray,   # (N,2) 계획 경로
                base_speed: float,  # m/s 기준 속도
                ) -> tuple[float, float, np.ndarray | None]:
        """
        Returns
        -------
        v_left   : float  왼쪽 바퀴 속도 (m/s)
        v_right  : float  오른쪽 바퀴 속도 (m/s)
        target   : (2,)   룩어헤드 포인트
        """
        target = self._find_lookahead(pos, path)
        if target is None:
            return 0.0, 0.0, None

        # 목표점 → 차량 로컬 프레임
        dx =  (target[0] - pos[0]) * math.cos(-yaw) \
            - (target[1] - pos[1]) * math.sin(-yaw)
        dy =  (target[0] - pos[0]) * math.sin(-yaw) \
            + (target[1] - pos[1]) * math.cos(-yaw)

        # Pure Pursuit 조향각
        ld_actual = max(math.hypot(dx, dy), 1e-3)
        alpha     = math.atan2(dy, dx)
        steer     = math.atan2(2.0 * self.track_width * math.sin(alpha),
                               ld_actual)
        steer     = max(-self.max_steer, min(self.max_steer, steer))

        # 차동구동 속도 계산
        # 각속도 ω = v * tan(δ) / L
        omega   = base_speed * math.tan(steer) / max(self.track_width, 1e-3)
        v_left  = base_speed - omega * self.track_width / 2.0
        v_right = base_speed + omega * self.track_width / 2.0

        # 최대 속도 정규화
        max_v = max(abs(v_left), abs(v_right), 1e-6)
        if max_v > abs(base_speed) and abs(base_speed) > 0:
            scale   = abs(base_speed) / max_v
            v_left  *= scale
            v_right *= scale

        log.debug(f"PP α={math.degrees(alpha):.1f}° δ={math.degrees(steer):.1f}°"
                  f" vL={v_left:.3f} vR={v_right:.3f}")

        return float(v_left), float(v_right), target

    # ── HUD 표시용 등가 조향각만 반환 ─────────────────
    def compute_steering(self,
                         pos, yaw, path
                         ) -> tuple[float, np.ndarray | None]:
        target = self._find_lookahead(pos, path)
        if target is None:
            return 0.0, None

        dx =  (target[0] - pos[0]) * math.cos(-yaw) \
            - (target[1] - pos[1]) * math.sin(-yaw)
        dy =  (target[0] - pos[0]) * math.sin(-yaw) \
            + (target[1] - pos[1]) * math.cos(-yaw)

        ld_actual = max(math.hypot(dx, dy), 1e-3)
        alpha     = math.atan2(dy, dx)

        # track_width 대신 등가 축거로 조향각 계산 (Pure Pursuit 원식)
        steer = math.atan2(2.0 * self.track_width * math.sin(alpha), ld_actual)
        steer = max(-self.max_steer, min(self.max_steer, steer))
        return float(math.degrees(steer)), target

    # ──────────────────────────────────────────────────
    def _find_lookahead(self, pos, path):
        for pt in path:
            if np.linalg.norm(pt - pos) >= self.ld:
                return pt
        return path[-1] if len(path) else None

    # ── 유틸: m/s → RPM 변환 (arduino_comm 없이도 사용) ─
    @staticmethod
    def mps_to_rpm(v_ms: float, max_rpm: int = 150) -> int:
        rpm = (v_ms / WHEEL_CIRC) * 60.0
        return int(max(-max_rpm, min(max_rpm, rpm)))
