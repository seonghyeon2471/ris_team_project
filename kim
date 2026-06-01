#!/usr/bin/env python3
"""
LiDAR FGM 시각화 테스트 (실제 LiDAR 없이 시뮬레이션)
color_tracking_robot.py 의 FGM 클래스를 그대로 임포트해서 테스트합니다.
"""

import numpy as np
import math
import time

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False
    print("[WARN] matplotlib unavailable – text output only")

# ── FGM 로직 복사 (독립 실행용) ──────────────────────────────────────────────
MAX_LIDAR_DIST = 3500
FGM_SAFE_DIST  = 400
FOV            = 180

def fgm_process(scan_dict):
    if not scan_dict:
        return 0.0, 9999.0
    angles    = np.arange(-FOV//2, FOV//2 + 1, dtype=float)
    distances = np.full(len(angles), MAX_LIDAR_DIST, dtype=float)
    for raw_ang, dist in scan_dict.items():
        ang = raw_ang if raw_ang <= 180 else raw_ang - 360
        if dist <= 0 or dist > MAX_LIDAR_DIST:
            dist = MAX_LIDAR_DIST
        idx = int(round(ang)) + FOV // 2
        if 0 <= idx < len(distances):
            distances[idx] = min(distances[idx], dist)
    obstacle_mask = distances < FGM_SAFE_DIST
    bubble_mask   = np.zeros_like(obstacle_mask)
    bubble_r = 5
    for i in np.where(obstacle_mask)[0]:
        lo = max(0, i - bubble_r)
        hi = min(len(distances)-1, i + bubble_r)
        bubble_mask[lo:hi+1] = True
    safe_mask = ~bubble_mask
    front_lo = FOV // 2 - 30
    front_hi = FOV // 2 + 30
    min_front = float(np.min(distances[front_lo:front_hi+1]))
    gaps, in_gap, gap_start = [], False, 0
    for i, safe in enumerate(safe_mask):
        if safe and not in_gap:
            gap_start, in_gap = i, True
        elif not safe and in_gap:
            gaps.append((gap_start, i-1)); in_gap = False
    if in_gap:
        gaps.append((gap_start, len(safe_mask)-1))
    if not gaps:
        return 0.0, min_front
    best_gap = max(gaps, key=lambda g: g[1]-g[0])
    gap_center_idx = (best_gap[0] + best_gap[1]) // 2
    return float(angles[gap_center_idx]), min_front

# ── 시뮬레이션 스캔 생성 ──────────────────────────────────────────────────────
def make_scan(obstacle_angles=None, obstacle_dist=300):
    """장애물이 없는 빈 방 + 지정 방향 장애물"""
    scan = {}
    for a in range(0, 360, 2):
        scan[a] = 1500  # 기본 1.5m
    if obstacle_angles:
        for a in obstacle_angles:
            for da in range(-5, 6):
                scan[(a + da) % 360] = obstacle_dist
    return scan

def run_tests():
    tests = [
        ("No obstacles",       {},          None,     1500),
        ("Front obstacle",     {0: 300},    range(-15, 16), 300),
        ("Right obstacle",     {30: 250},   range(25, 36), 250),
        ("Left obstacle",      {-20: 200},  range(-25, -14), 200),
        ("Both sides",         {20: 280, -25: 260}, list(range(15,26))+list(range(-30,-14)), 260),
    ]
    print(f"{'Test':<25} {'safe_angle':>12} {'min_front':>12}")
    print("-" * 52)
    for name, _, obs, dist in tests:
        scan = make_scan(obs, dist)
        sa, mf = fgm_process(scan)
        print(f"{name:<25} {sa:>+11.1f}°  {mf:>10.0f}mm")

    if HAS_MPL:
        fig, axes = plt.subplots(1, 3, figsize=(14, 5), subplot_kw=dict(polar=True))
        scenarios = [
            ("No obstacles",   None,         1500),
            ("Front obstacle", range(-15,16), 300),
            ("Right obstacle", range(25,36),  250),
        ]
        for ax, (title, obs, dist) in zip(axes, scenarios):
            scan = make_scan(obs, dist)
            thetas = [math.radians(a if a <= 180 else a-360) for a in scan]
            rs     = [scan[a]/1000 for a in scan]
            ax.scatter(thetas, rs, s=2, c="steelblue")
            sa, _ = fgm_process(scan)
            ax.plot([0, math.radians(sa)], [0, 1.8], "r-", lw=2, label=f"safe={sa:.0f}°")
            ax.set_title(title, pad=12)
            ax.set_theta_zero_location("N")
            ax.set_theta_direction(-1)
            ax.legend(loc="lower right", fontsize=8)
        plt.tight_layout()
        plt.savefig("/tmp/fgm_test.png", dpi=100)
        print("\n[INFO] Plot saved to /tmp/fgm_test.png")
        plt.show()

if __name__ == "__main__":
    run_tests()
