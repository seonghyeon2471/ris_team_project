import time

from lidar import SimpleLidar
from mapping import LocalMapper
from planner import LocalPathPlanner
from motor import MotorController

# =========================
# INIT
# =========================

lidar  = SimpleLidar()
mapper = LocalMapper()
planner = LocalPathPlanner()
motor  = MotorController()

print("SYSTEM START")

# =========================
# LOOP
# =========================

try:

    while True:

        scan = lidar.read_scan()

        if len(scan) == 0:
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

        time.sleep(0.03)

except KeyboardInterrupt:

    print("STOP")
    motor.stop()
    lidar.stop()
