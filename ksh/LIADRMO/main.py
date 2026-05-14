import time

from lidar import SimpleLidar
from motor import MotorController
from planner import GapPlanner

print("SYSTEM START")

# =========================
# INIT
# =========================

lidar = SimpleLidar()

motor = MotorController()

planner = GapPlanner()

# =========================
# LOOP
# =========================

counter = 0

try:

    while True:

        scan = lidar.read_scan()

        if len(scan) == 0:
            continue

        v, w = planner.compute_control(scan)

        motor.send(v, w)

        counter += 1

        time.sleep(0.02)

except KeyboardInterrupt:

    print("STOP")

    motor.stop()

    lidar.stop()
