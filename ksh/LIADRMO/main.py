import time

from lidar import SimpleLidar
from motor import MotorController
from planner import GapPlanner
from mapping import SimpleMapper

print("SYSTEM START")

# =========================
# INIT
# =========================

lidar = SimpleLidar()

motor = MotorController()

planner = GapPlanner()

mapper = SimpleMapper()

# =========================
# LOOP
# =========================

counter = 0

try:

    while True:

        scan = lidar.read_scan()

        if len(scan) == 0:
            continue

        # mapper.update(scan)

        v, w = planner.compute_control(scan)

        motor.send(v, w)

        counter += 1

        if counter % 5 == 0:

            print(
                f"SCAN:{len(scan)} "
                f"v:{v:.2f} "
                f"w:{w:.2f}"
            )

        time.sleep(0.02)

except KeyboardInterrupt:

    print("STOP")

    motor.stop()

    lidar.stop()
