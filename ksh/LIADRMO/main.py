import time

from lidar import SimpleLidar
from mapping import LocalMapper
from planner import LocalPathPlanner
from motor import MotorController

# =========================
# INIT
# =========================

lidar   = SimpleLidar()
mapper  = LocalMapper()
planner = LocalPathPlanner()
motor   = MotorController()

print("SYSTEM START")

# =========================
# LOOP
# =========================

try:

    while True:

        scan = lidar.read_scan()

        # 스캔 포인트가 너무 적으면 신뢰도 낮으므로 스킵
        if len(scan) < 20:
            continue

        # 1) 맵 업데이트 (inflated costmap 포함)
        mapper.update(scan)

        # 2) 경로 계획 + 속도 계산 (맵 전달)
        v, w = planner.compute_control(scan, mapper)

        # 3) 모터 명령
        motor.send(v, w)

        # 4) 디버그 출력
        path, wp_idx = planner.get_path()
        print(
            f"SCAN:{len(scan):3d} "
            f"v:{v:.2f} w:{w:.2f} | "
            f"PATH:{len(path)} WP:{wp_idx}"
        )

        # 0.03 → 0.05: 라이다 데이터 생성 속도에 맞춰 루프 속도 낮춤
        time.sleep(0.05)

except KeyboardInterrupt:

    print("STOP")
    motor.stop()
    lidar.stop()
