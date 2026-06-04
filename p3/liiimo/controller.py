"""
Pure Pursuit Controller — 차동구동(Differential Drive) 버전
============================================================
조향각 대신 좌/우 모터 속도 비율을 출력합니다.

Reference: arXiv:2507.12449 §II-B2
"""

import logging
import numpy as np

log = logging.getLogger("pure_pursuit")


class PurePursuitController:
    def __init__(self, cfg: dict):
        self.L           = cfg["wheelbase"]        # 좌우 바퀴 간격 (m) — 트랙 폭
        self.ld          = cfg["lookahead_dist"]   # 전방 주시 거리 (m)
        self.max_steer   = np.radians(cfg["max_steer_deg"])
        self.cruise_spd  = cfg["cruise_speed"]
        self.avoid_spd   = cfg["avoidance_speed"]

    def compute(self,
                pos: np.ndarray,    # (2,) 차량 위치 (vehicle frame)
                yaw: float,         # 헤딩 (rad)
                path: np.ndarray,   # (N,2) 계획 경로
                base_speed: float,  # m/s 기본 속도
                ) -> tuple[float, float, np.ndarray | None]:
        """
        Returns
        -------
        speed_left  : float   왼쪽 모터 명령 속도 (m/s)
        speed_right : float   오른쪽 모터 명령 속도 (m/s)
        target_pt   : (2,)    룩어헤드 포인트 (시각화용)
        """
        target = self._find_lookahead(pos, path)
        if target is None:
            return 0.0, 0.0, None

        # ── 목표점을 차량 로컬 프레임으로 변환 ────────
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        # -yaw 회전
        tx =  dx * np.cos(-yaw) - dy * np.sin(-yaw)
        ty =  dx * np.sin(-yaw) + dy * np.cos(-yaw)

        # ── Pure Pursuit 조향각 계산 ───────────────────
        ld_actual = max(np.hypot(tx, ty), 1e-3)
        alpha     = np.arctan2(ty, tx)
        # δ = arctan(2·L·sin(α) / ld)
        steer = np.arctan2(2.0 * self.L * np.sin(alpha), ld_actual)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        # ── 차동구동 속도 믹싱 ─────────────────────────
        # 곡률 ρ = tan(δ) / L
        # vL = v·(1 - L·ρ/2),  vR = v·(1 + L·ρ/2)
        curvature = np.tan(steer) / max(self.L, 1e-3)
        half_LK   = 0.5 * self.L * curvature

        v_left  = base_speed * (1.0 - half_LK)
        v_right = base_speed * (1.0 + half_LK)

        # ── 최대 속도 정규화 ───────────────────────────
        max_v = max(abs(v_left), abs(v_right), 1e-6)
        if max_v > base_speed:
            v_left  *= base_speed / max_v
            v_right *= base_speed / max_v

        steer_deg = float(np.degrees(steer))
        log.debug(f"PP steer={steer_deg:.1f}° "
                  f"vL={v_left:.3f} vR={v_right:.3f}")

        return float(v_left), float(v_right), target

    # ── 하위 호환용 래퍼 (main.py의 steer_deg 인터페이스 유지) ──
    def compute_steering(self,
                         pos, yaw, path
                         ) -> tuple[float, np.ndarray | None]:
        """steer_deg와 target만 반환 (main.py HUD 표시용)."""
        target = self._find_lookahead(pos, path)
        if target is None:
            return 0.0, None

        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        tx =  dx * np.cos(-yaw) - dy * np.sin(-yaw)
        ty =  dx * np.sin(-yaw) + dy * np.cos(-yaw)

        ld_actual = max(np.hypot(tx, ty), 1e-3)
        alpha     = np.arctan2(ty, tx)
        steer     = np.arctan2(2.0 * self.L * np.sin(alpha), ld_actual)
        steer     = float(np.clip(steer, -self.max_steer, self.max_steer))
        return float(np.degrees(steer)), target

    # ──────────────────────────────────────────────────
    def _find_lookahead(self, pos, path):
        """룩어헤드 거리 이상의 첫 번째 경로점 반환."""
        for pt in path:
            if np.linalg.norm(pt - pos) >= self.ld:
                return pt
        return path[-1] if len(path) else None
